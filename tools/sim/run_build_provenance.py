#!/usr/bin/env python3
"""Check build and package identity."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("build_provenance", ROOT / "scripts/build_provenance.py")
provenance = importlib.util.module_from_spec(spec)
spec.loader.exec_module(provenance)


def rejects(action, message):
    try:
        action()
    except (ValueError, OSError, KeyError, zipfile.BadZipFile):
        return
    raise AssertionError(message)


with tempfile.TemporaryDirectory(prefix="gamecom-provenance-") as temporary:
    repo = Path(temporary) / "repo"
    repo.mkdir()
    build = repo / "build/gamecom"
    source = repo / "source.sv"
    source.write_text("module original; endmodule\n")
    (repo / ".gitignore").write_text("/build/\n")
    for relative in ("tools/podman/build.sh", "scripts/build_provenance.py"):
        target = repo / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / relative, target)
    shutil.copytree(ROOT / "pkg", repo / "pkg")
    def git(*arguments):
        return subprocess.check_output(["git", "-c", "commit.gpgsign=false", "-c", "core.hooksPath=/dev/null",
                                        "-C", str(repo), *arguments], stderr=subprocess.DEVNULL)
    git("init", "-q")
    git("add", ".")
    git("-c", "user.name=Provenance Test", "-c", "user.email=test@example.invalid", "commit", "-qm", "fixture")
    settings = {"IMAGE": "localhost/pocket-quartus:25.1std", "SEED": "", "FITTER_EFFORT": "", "NPROC": ""}
    provenance.begin(repo, build, settings)
    raw_rbf = bytes(range(256))
    for kind, relative in provenance.ARTIFACTS.items():
        target = build / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(raw_rbf if kind == "rbf" else
                           b"Fitter Status : Successful - fixture\n" if kind == "fit" else
                           b"Type : Slow Model Setup 'clock'\nSlack : 0.100\n")
    sealed = provenance.seal(repo, build)
    assert sealed["source"]["dirty"] is False
    assert provenance.verify(repo, build) == sealed
    rejects(lambda: provenance.verify(repo, build, {"SEED": "2"}), "Accepted a different fitter setting")

    for kind, relative in provenance.ARTIFACTS.items():
        target = build / relative
        old = target.read_bytes()
        target.write_bytes(old + b"changed")
        rejects(lambda: provenance.verify(repo, build), f"Accepted a changed compiled {kind}")
        target.write_bytes(old)
    source.write_text("module modified; endmodule\n")
    rejects(lambda: provenance.verify(repo, build), "Accepted changed source under the same HEAD")

    sentinel = build / "work/src/sentinel"
    sentinel.parent.mkdir(parents=True, exist_ok=True)
    sentinel.write_text("preserve staged source")
    old_package = build / "keep.zip"
    old_package.write_bytes(b"previous package")
    (build / "report.txt").write_text("previous report")
    python = build / "venv/bin/python3"
    python.parent.mkdir(parents=True, exist_ok=True)
    python.symlink_to(sys.executable)
    environment = dict(os.environ, SKIP_COMPILE="1", SEED="", FITTER_EFFORT="", NPROC="",
                       IMAGE=settings["IMAGE"])
    result = subprocess.run(["bash", str(repo / "tools/podman/build.sh")],
                            env=environment, capture_output=True, text=True)
    assert result.returncode != 0 and "Compiled source identity differs" in result.stderr, result.stderr
    assert sentinel.read_text() == "preserve staged source"
    assert old_package.read_bytes() == b"previous package"
    assert (build / "report.txt").read_text() == "previous report"

    source.write_text("module original; endmodule\n")
    (repo / "new-source.sv").write_text("module untracked; endmodule\n")
    rejects(lambda: provenance.verify(repo, build), "Accepted an untracked source addition")
    (repo / "new-source.sv").unlink()
    git("-c", "user.name=Provenance Test", "-c", "user.email=test@example.invalid", "commit", "--allow-empty", "-qm", "new HEAD")
    rejects(lambda: provenance.verify(repo, build), "Relabeled existing hardware under a new commit")

    source.write_text("module dirty_but_compiled; endmodule\n")
    provenance.begin(repo, build, settings)
    source.write_text("module changed_during_compile; endmodule\n")
    rejects(lambda: provenance.seal(repo, build), "Sealed a snapshot modified during compilation")
    assert not (build / "compile-provenance.json").exists()
    source.write_text("module dirty_but_compiled; endmodule\n")
    sealed = provenance.seal(repo, build)
    assert sealed["source"]["dirty"] is True
    (build / "report.txt").write_text(f"core: {provenance.CORE}\ncommit: {sealed['source']['short_commit']}\nSetup 0.100 ns\n")
    archive = build / "kroy.GameCom_preview.zip"
    def make_package(version, bitstream=raw_rbf.translate(provenance.REVERSE), extra=None, changes=None):
        with zipfile.ZipFile(archive, "w") as output:
            for path in (repo / "pkg").rglob("*"):
                if path.is_file():
                    relative = path.relative_to(repo / "pkg").as_posix()
                    data = path.read_bytes()
                    if relative == f"Cores/{provenance.CORE}/core.json":
                        metadata = json.loads(data)
                        metadata["core"]["metadata"]["version"] = version
                        data = json.dumps(metadata).encode()
                    if relative in (changes or {}):
                        data = changes[relative]
                        if data is None:
                            continue
                    output.writestr(relative, data)
            output.writestr(provenance.RBF_MEMBER, bitstream)
            if extra:
                output.writestr(*extra)
    for version in ("preview-1", f"0.9999.{sealed['source']['short_commit']}.dirty"):
        make_package(version)
        manifest = provenance.package_manifest(repo, build, archive)
        assert provenance.validate_package(archive) == manifest

    required_definitions = ("core", "audio", "data", "input", "interact", "variants", "video")
    assert provenance.CORE_DEFINITIONS == required_definitions
    for root in required_definitions:
        member = f"Cores/{provenance.CORE}/{root}.json"
        valid = json.loads((repo / "pkg" / member).read_bytes())
        bad_magic = json.loads(json.dumps(valid))
        bad_magic[root]["magic"] = "APF_VER_2"
        for label, contents in (
                ("missing file", None),
                ("malformed JSON", b"{"),
                ("wrong root", json.dumps({"unrelated": valid[root]}).encode()),
                ("non-object root", json.dumps({root: []}).encode()),
                ("bad magic", json.dumps(bad_magic).encode())):
            make_package("preview-1", changes={member: contents})
            rejects(lambda: provenance.package_manifest(repo, build, archive),
                    f"Accepted {label} in required definition {root}.json")
            assert provenance.verify(repo, build) == sealed
    member = f"Cores/{provenance.CORE}/core.json"
    for field, value in (("shortname", "Game.com"), ("author", "other")):
        invalid = json.loads((repo / "pkg" / member).read_bytes())
        invalid["core"]["metadata"][field] = value
        make_package("preview-1", changes={member: json.dumps(invalid).encode()})
        rejects(lambda: provenance.package_manifest(repo, build, archive),
                f"Accepted core folder mismatch for {field}")
        assert provenance.verify(repo, build) == sealed
    make_package("preview-1")
    manifest = provenance.package_manifest(repo, build, archive)
    assert provenance.validate_package(archive) == manifest
    make_package("preview-1", b"wrong binary")
    rejects(lambda: provenance.package_manifest(repo, build, archive), "Accepted an unrelated reversed RBF")
    make_package("preview-1")
    provenance.package_manifest(repo, build, archive)
    make_package("preview-1", b"same version, altered binary")
    rejects(lambda: provenance.validate_package(archive), "Accepted altered ZIP bytes with unchanged version")
    make_package("preview-1")
    provenance.package_manifest(repo, build, archive)
    report_path = build / "report.txt"
    old_report = report_path.read_bytes()
    report_path.write_bytes(old_report + b"different evidence")
    rejects(lambda: provenance.validate_package(archive), "Accepted a different timing report")
    report_path.write_bytes(old_report)
    (build / "TIMING_FAILED").write_text("failed")
    rejects(lambda: provenance.validate_package(archive), "Accepted a timing failure marker")
    (build / "TIMING_FAILED").unlink()
    make_package("preview-1", extra=("../escape", b"bad"))
    rejects(lambda: provenance.inspect_package(archive), "Accepted ZIP path traversal")
    make_package("preview-1")
    provenance.package_manifest(repo, build, archive)

print("PASS compile provenance: source/commit/settings/RBF/timing binding, SKIP_COMPILE fails before mutation")
print("PASS package: release and dirty versions, ZIP/report binding")
print("PASS required APF definitions: seven files, JSON objects, matching roots and magic, unchanged compiled source")
