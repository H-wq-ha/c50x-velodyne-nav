"""Safe file lifecycle helpers for semantic and RTAB-Map SQLite databases."""

from __future__ import annotations

import os
import stat
from typing import Iterable


SIDECAR_SUFFIXES = ('-wal', '-shm', '-journal')


class DatabaseLifecycleError(ValueError):
    """Raised when database paths are unsafe or cannot be prepared."""


def _normalize_path(path: str) -> str:
    if not path or not str(path).strip():
        raise DatabaseLifecycleError('database path must be non-empty')
    return os.path.abspath(os.path.expanduser(str(path).strip()))


def _canonical_path(path: str) -> str:
    return os.path.normcase(os.path.realpath(path))


def _database_files(path: str) -> tuple[str, ...]:
    return (path,) + tuple(path + suffix for suffix in SIDECAR_SUFFIXES)


def _paths_are_samefile(first: str, second: str) -> bool:
    if _canonical_path(first) == _canonical_path(second):
        return True
    try:
        return os.path.samefile(first, second)
    except (FileNotFoundError, NotADirectoryError):
        return False


def _validate_recreatable_file(path: str) -> None:
    """Allow an absent file or a regular file; reject links and other types."""
    if not os.path.lexists(path):
        return
    try:
        info = os.lstat(path)
    except OSError as exc:
        raise DatabaseLifecycleError(f'cannot inspect database path {path}: {exc}') from exc
    if not stat.S_ISREG(info.st_mode):
        raise DatabaseLifecycleError(
            f'database path must be an absent or regular file: {path}')


def _validate_parent_path(path: str) -> None:
    parent = os.path.dirname(path)
    current = parent
    while not os.path.lexists(current):
        ancestor = os.path.dirname(current)
        if ancestor == current:
            raise DatabaseLifecycleError(f'no existing parent directory for {path}')
        current = ancestor
    if not os.path.isdir(current):
        raise DatabaseLifecycleError(
            f'database parent is not a directory: {current}')
    if not os.access(current, os.W_OK | os.X_OK):
        raise DatabaseLifecycleError(
            f'database parent is not writable/searchable: {current}')


def _database_path_sets_overlap(first: str, second: str) -> bool:
    first_set = {_canonical_path(path) for path in _database_files(first)}
    second_set = {_canonical_path(path) for path in _database_files(second)}
    if first_set.intersection(second_set):
        return True
    for first_file in _database_files(first):
        for second_file in _database_files(second):
            if _paths_are_samefile(first_file, second_file):
                return True
    return False


def validate_database_pair(rtabmap_path: str, semantic_path: str) -> tuple[str, str]:
    """Validate two distinct SQLite targets and their sidecars without mutation."""
    rtabmap = _normalize_path(rtabmap_path)
    semantic = _normalize_path(semantic_path)
    if _database_path_sets_overlap(rtabmap, semantic):
        raise DatabaseLifecycleError(
            'RTAB-Map and semantic database paths (including SQLite sidecars) '
            'must refer to different files')

    all_files = _database_files(rtabmap) + _database_files(semantic)
    for path in all_files:
        _validate_recreatable_file(path)
        _validate_parent_path(path)
    return rtabmap, semantic


def _open_file_targets(targets: Iterable[str]) -> set[str]:
    """Return target paths currently held open, using /proc fd links/inodes only."""
    proc_root = '/proc'
    if not os.path.isdir(proc_root):
        return set()

    normalized_targets = {_canonical_path(path): path for path in targets}
    existing_inodes = {}
    for canonical, path in normalized_targets.items():
        try:
            info = os.stat(path)
        except OSError:
            continue
        existing_inodes[(info.st_dev, info.st_ino)] = canonical

    opened: set[str] = set()
    try:
        process_ids = os.listdir(proc_root)
    except OSError:
        return opened
    for process_id in process_ids:
        if not process_id.isdigit():
            continue
        fd_directory = os.path.join(proc_root, process_id, 'fd')
        try:
            fd_names = os.listdir(fd_directory)
        except OSError:
            continue
        for fd_name in fd_names:
            fd_path = os.path.join(fd_directory, fd_name)
            try:
                link_target = os.readlink(fd_path)
                if link_target.endswith(' (deleted)'):
                    link_target = link_target[:-10]
                canonical_link = _canonical_path(link_target)
                if canonical_link in normalized_targets:
                    opened.add(normalized_targets[canonical_link])
                    continue
                info = os.stat(fd_path)
            except OSError:
                continue
            canonical_inode = existing_inodes.get((info.st_dev, info.st_ino))
            if canonical_inode is not None:
                opened.add(normalized_targets[canonical_inode])
    return opened


def _ensure_parent_directories(paths: Iterable[str]) -> None:
    for path in paths:
        os.makedirs(os.path.dirname(path), exist_ok=True)


def _remove_database_files(paths: Iterable[str]) -> None:
    for database_path in paths:
        for path in _database_files(database_path):
            if os.path.lexists(path):
                os.remove(path)


def prepare_mapping_databases(
    rtabmap_path: str,
    semantic_path: str,
    *,
    overwrite_databases: bool = True,
) -> tuple[str, str]:
    """Validate both mapping databases, then optionally clear exactly those files.

    All path/type/in-use checks happen for both databases before any file is
    removed. Only each database and its SQLite WAL/SHM/journal files are touched.
    """
    rtabmap, semantic = validate_database_pair(rtabmap_path, semantic_path)
    target_files = _database_files(rtabmap) + _database_files(semantic)

    open_targets = _open_file_targets(target_files)
    if open_targets:
        busy = ', '.join(sorted(open_targets))
        raise DatabaseLifecycleError(
            'cannot reset database files while a process has them open; '
            f'stop the existing mapping process first: {busy}')

    existing = [path for path in target_files if os.path.lexists(path)]
    if existing and not overwrite_databases:
        raise DatabaseLifecycleError(
            'database files already exist and overwrite_databases is false: '
            + ', '.join(existing))

    _ensure_parent_directories((rtabmap, semantic))
    if overwrite_databases:
        _remove_database_files((rtabmap, semantic))
    return rtabmap, semantic


def prepare_databases(
    rtabmap_path: str,
    semantic_path: str,
    *,
    start_rtabmap: bool,
    mode: str,
    overwrite_databases: bool,
) -> tuple[str, str]:
    """Apply mapping reset policy; localization and attach are never destructive."""
    rtabmap, semantic = validate_database_pair(rtabmap_path, semantic_path)
    if not start_rtabmap:
        return rtabmap, semantic
    if mode == 'localization':
        if not os.path.isfile(rtabmap):
            raise DatabaseLifecycleError(
                f'localization RTAB-Map database does not exist: {rtabmap}')
        _ensure_parent_directories((rtabmap, semantic))
        return rtabmap, semantic
    if mode != 'mapping':
        raise DatabaseLifecycleError("mode must be 'mapping' or 'localization'")
    return prepare_mapping_databases(
        rtabmap, semantic, overwrite_databases=overwrite_databases)
