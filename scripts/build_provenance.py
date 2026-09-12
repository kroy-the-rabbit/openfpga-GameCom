#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Bind packaged bitstreams to the source snapshot and timing evidence compiled."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import zipfile

CORE = "kroy.GameCom"
RBF_MEMBER = f"Cores/{CORE}/bitstream.rbf_r"
CORE_DEFINITIONS = ("core", "audio", "data", "input", "interact", "variants", "video")
ARTIFACTS = {
    "rbf": "work/src/fpga/build/output_files/ap_core.rbf",
    "fit": "work/src/fpga/build/output_files/ap_core.fit.summary",
    "sta": "work/src/fpga/build/output_files/ap_core.sta.summary",
}
REVERSE = bytes(int(f"{value:08b}"[::-1], 2) for value in range(256))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def file_digest(path):
    return digest(Path(path).read_bytes())


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")
    temporary.replace(path)


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args])


def snapshot(repo):
    """Hash tracked and nonignored files, including local changes and deletions."""
    repo = Path(repo)
    paths = sorted(set(git(repo, "ls-files", "-z", "--cached", "--others", "--exclude-standard").split(b"\0")) - {b""})
    hashes = {}
    for raw_name in paths:
        name = raw_name.decode("utf-8")
        path = repo / name
        if path.is_symlink():
            value = b"symlink\0" + str(path.readlink()).encode()
        elif path.is_file():
            value = b"file\0" + path.read_bytes()
        elif not path.exists():
            value = b"deleted\0"
        else:
            raise ValueError(f"Unsupported source entry: {name}")
        hashes[name] = digest(value)
    return {
        "commit": git(repo, "rev-parse", "HEAD").decode().strip(),
        "short_commit": git(repo, "rev-parse", "--short", "HEAD").decode().strip(),
        "dirty": bool(git(repo, "status", "--porcelain").strip()),
        "source_sha256": digest(json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode()),
    }


def begin(repo, build, settings):
    build = Path(build)
    build.mkdir(parents=True, exist_ok=True)
    write_json(build / "compile-pending.json", {
        "schema_version": 1, "source": snapshot(repo), "settings": settings,
    })
    for name in ("compile-provenance.json", "package-manifest.json"):
        (build / name).unlink(missing_ok=True)


def artifact_hashes(build):
    build = Path(build)
    values = {}
    for name, relative in ARTIFACTS.items():
        path = build / relative
        if not path.is_file() or path.stat().st_size == 0:
            raise ValueError(f"Missing compiled artifact: {relative}")
        values[name] = {"path": relative, "sha256": file_digest(path)}
    if not re.search(r"Fitter Status\s*:\s*Successful", (build / ARTIFACTS["fit"]).read_text()):
        raise ValueError("Cannot seal an unsuccessful fit")
    return values


def seal(repo, build):
    build = Path(build)
    pending = read_json(build / "compile-pending.json")
    if pending.get("schema_version") != 1 or pending.get("source") != snapshot(repo):
        raise ValueError("Source snapshot changed while compiling; refusing to label its output")
    pending["artifacts"] = artifact_hashes(build)
    write_json(build / "compile-provenance.json", pending)
    (build / "compile-pending.json").unlink()
    return pending


def verify(repo, build, settings=None):
    build = Path(build)
    provenance = read_json(build / "compile-provenance.json")
    if provenance.get("schema_version") != 1 or provenance.get("source") != snapshot(repo):
        raise ValueError("Compiled source identity differs from this checkout; run a fresh compile")
    if provenance.get("artifacts") != artifact_hashes(build):
        raise ValueError("Compiled RBF or timing summaries differ from their recorded hashes")
    for key, value in (settings or {}).items():
        if value and provenance.get("settings", {}).get(key) != value:
            raise ValueError(f"Requested {key} differs from the recorded compile setting")
    return provenance


def inspect_package(package):
    """Validate Pocket package contents."""
    with zipfile.ZipFile(package) as archive:
        names = set()
        for member in archive.infolist():
            name = member.filename
            parts = PurePosixPath(name).parts
            if (not parts or name.startswith("/") or "\\" in name or ".." in parts
                    or stat.S_ISLNK(member.external_attr >> 16)):
                raise ValueError(f"Unsafe package entry: {name}")
            if name in names:
                raise ValueError(f"Duplicate package entry: {name}")
            names.add(name)
            if parts[0] not in {"Assets", "Cores", "Platforms"}:
                raise ValueError(f"Unexpected package root: {name}")
            if parts[0] == "Cores" and len(parts) > 1 and parts[1] != CORE:
                raise ValueError(f"Unexpected packaged core: {name}")
            if parts[0] == "Assets" and len(parts) > 1 and parts[1] != "gamecom":
                raise ValueError(f"Unexpected packaged asset tree: {name}")
            if parts[0] == "Platforms" and len(parts) > 1 and parts[1] != "gamecom.json":
                raise ValueError(f"Unexpected packaged platform: {name}")
        definitions = {}
        for root in CORE_DEFINITIONS:
            name = f"Cores/{CORE}/{root}.json"
            try:
                contents = archive.read(name)
            except KeyError as exc:
                raise ValueError(f"Missing required core definition: {name}") from exc
            try:
                document = json.loads(contents)
            except ValueError as exc:
                raise ValueError(f"Invalid JSON in required core definition: {name}") from exc
            if not isinstance(document, dict) or not isinstance(document.get(root), dict):
                raise ValueError(f"Required core definition must contain a '{root}' object: {name}")
            if document[root].get("magic") != "APF_VER_1":
                raise ValueError(f"Required core definition has invalid magic: {name}")
            definitions[root] = document[root]
        metadata = definitions["core"]
        identity = metadata["metadata"]
        if f"{identity.get('author')}.{identity.get('shortname')}" != CORE:
            raise ValueError("Core folder must match metadata author.shortname")
        slots = definitions["data"]["data_slots"]
        platform = json.loads(archive.read("Platforms/gamecom.json"))["platform"]
        if (metadata["metadata"]["platform_ids"] != ["gamecom"]
                or metadata["cores"][0]["filename"] != "bitstream.rbf_r"
                or [slot["id"] for slot in slots] != [1, 4]
                or platform["name"] != "Game.com"):
            raise ValueError("Package metadata does not identify the Game.com SD-ROM core")
        bitstream = archive.read(RBF_MEMBER)
        if not bitstream:
            raise ValueError("Package bitstream is empty")
        return metadata["metadata"]["version"], bitstream


def package_manifest(repo, build, package):
    build, package = Path(build), Path(package)
    provenance = verify(repo, build)
    version, reversed_rbf = inspect_package(package)
    if reversed_rbf != (build / ARTIFACTS["rbf"]).read_bytes().translate(REVERSE):
        raise ValueError("Packaged bitstream is not the reversed compiled RBF")
    report = (build / "report.txt").read_text()
    if f"commit: {provenance['source']['short_commit']}\n" not in report:
        raise ValueError("Timing report commit differs from the compiled source")
    if (build / "TIMING_FAILED").exists():
        raise ValueError("Timing failure marker prevents packaging")
    manifest = {
        "schema_version": 1,
        "core": CORE,
        "source": provenance["source"],
        "menu_version": version,
        "compile_provenance_sha256": file_digest(build / "compile-provenance.json"),
        "package": {"filename": package.name, "sha256": file_digest(package)},
        "bitstream": {"filename": RBF_MEMBER, "sha256": digest(reversed_rbf)},
        "report_sha256": file_digest(build / "report.txt"),
    }
    write_json(build / "package-manifest.json", manifest)
    return manifest


def validate_package(package):
    package = Path(package)
    build = package.parent
    manifest = read_json(build / "package-manifest.json")
    provenance = read_json(build / "compile-provenance.json")
    if (manifest.get("schema_version") != 1 or manifest.get("core") != CORE
            or provenance.get("schema_version") != 1
            or manifest.get("source") != provenance.get("source")
            or manifest.get("compile_provenance_sha256") != file_digest(build / "compile-provenance.json")
            or manifest.get("package") != {"filename": package.name, "sha256": file_digest(package)}
            or manifest.get("report_sha256") != file_digest(build / "report.txt")
            or (build / "TIMING_FAILED").exists()):
        raise ValueError("Package, compiled identity, or passing report does not match its manifest")
    version, bitstream = inspect_package(package)
    if (manifest.get("menu_version") != version
            or manifest.get("bitstream") != {"filename": RBF_MEMBER, "sha256": digest(bitstream)}):
        raise ValueError("Packaged core version or bitstream differs from its manifest")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name in ("begin", "seal", "verify", "identity", "package"):
        command = commands.add_parser(name)
        command.add_argument("repo", type=Path)
        command.add_argument("build", type=Path)
        if name in {"begin", "verify"}:
            command.add_argument("--setting", action="append", default=[])
        if name == "package":
            command.add_argument("archive", type=Path)
    commands.add_parser("validate-package").add_argument("archive", type=Path)
    args = parser.parse_args()
    try:
        settings = dict(value.split("=", 1) for value in getattr(args, "setting", []))
        if args.command == "begin":
            begin(args.repo, args.build, settings)
        elif args.command == "seal":
            seal(args.repo, args.build)
        elif args.command == "verify":
            verify(args.repo, args.build, settings)
        elif args.command == "identity":
            source = verify(args.repo, args.build)["source"]
            print(source["short_commit"], int(source["dirty"]))
        elif args.command == "package":
            package_manifest(args.repo, args.build, args.archive)
        else:
            validate_package(args.archive)
    except (ValueError, OSError, KeyError, IndexError, zipfile.BadZipFile, subprocess.CalledProcessError) as exc:
        parser.exit(1, f"Build provenance failed: {exc}\n")


if __name__ == "__main__":
    main()
