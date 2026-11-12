import os
import sqlite3
import subprocess
import sys

import pytest

from semantic_mapping.database_lifecycle import (
    DatabaseLifecycleError,
    prepare_databases,
    prepare_mapping_databases,
)


SIDECARS = ('-wal', '-shm', '-journal')


def _make_sentinel_database(path):
    connection = sqlite3.connect(path)
    try:
        connection.execute('CREATE TABLE sentinel (value TEXT NOT NULL)')
        connection.execute("INSERT INTO sentinel VALUES ('old-map')")
        connection.commit()
    finally:
        connection.close()


def _make_sidecars(path):
    for suffix in SIDECARS:
        with open(str(path) + suffix, 'wb') as stream:
            stream.write(('old-sidecar-' + suffix).encode())


def _contents(path):
    with open(path, 'rb') as stream:
        return stream.read()


def test_overwrite_removes_both_databases_and_all_sqlite_sidecars(tmp_path):
    rtabmap = tmp_path / 'rtabmap.db'
    semantic = tmp_path / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    _make_sentinel_database(semantic)
    _make_sidecars(rtabmap)
    _make_sidecars(semantic)

    prepare_mapping_databases(str(rtabmap), str(semantic), overwrite_databases=True)

    for path in (rtabmap, semantic):
        assert not path.exists()
        for suffix in SIDECARS:
            assert not os.path.lexists(str(path) + suffix)

    fresh = sqlite3.connect(semantic)
    try:
        tables = fresh.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='sentinel'"
        ).fetchall()
        assert tables == []
    finally:
        fresh.close()


@pytest.mark.parametrize('semantic_name', ['rtabmap.db', 'sub/../rtabmap.db'])
def test_same_path_or_realpath_alias_is_rejected_without_file_loss(tmp_path, semantic_name):
    rtabmap = tmp_path / 'rtabmap.db'
    _make_sentinel_database(rtabmap)
    before = _contents(rtabmap)

    with pytest.raises(DatabaseLifecycleError, match='different files'):
        prepare_mapping_databases(str(rtabmap), str(tmp_path / semantic_name))

    assert _contents(rtabmap) == before


def test_hardlink_samefile_alias_is_rejected_without_file_loss(tmp_path):
    rtabmap = tmp_path / 'rtabmap.db'
    semantic = tmp_path / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    os.link(rtabmap, semantic)
    before = _contents(rtabmap)

    with pytest.raises(DatabaseLifecycleError, match='different files'):
        prepare_mapping_databases(str(rtabmap), str(semantic))

    assert _contents(rtabmap) == before
    assert _contents(semantic) == before


def test_invalid_directory_target_rejects_before_either_database_is_removed(tmp_path):
    rtabmap = tmp_path / 'rtabmap.db'
    semantic_directory = tmp_path / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    semantic_directory.mkdir()
    before = _contents(rtabmap)

    with pytest.raises(DatabaseLifecycleError, match='regular file'):
        prepare_mapping_databases(str(rtabmap), str(semantic_directory))

    assert _contents(rtabmap) == before
    assert semantic_directory.is_dir()


def test_overwrite_false_refuses_existing_files_without_loss(tmp_path):
    rtabmap = tmp_path / 'rtabmap.db'
    semantic = tmp_path / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    _make_sentinel_database(semantic)
    before = (_contents(rtabmap), _contents(semantic))

    with pytest.raises(DatabaseLifecycleError, match='overwrite_databases is false'):
        prepare_mapping_databases(
            str(rtabmap), str(semantic), overwrite_databases=False)

    assert (_contents(rtabmap), _contents(semantic)) == before


def test_localization_and_attach_never_clear_databases_or_sidecars(tmp_path):
    rtabmap = tmp_path / 'rtabmap.db'
    semantic = tmp_path / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    _make_sentinel_database(semantic)
    _make_sidecars(rtabmap)
    _make_sidecars(semantic)
    targets = [rtabmap, semantic] + [
        tmp_path / (path.name + suffix)
        for path in (rtabmap, semantic)
        for suffix in SIDECARS
    ]
    before = {path: _contents(path) for path in targets}

    prepare_databases(
        str(rtabmap), str(semantic), start_rtabmap=True,
        mode='localization', overwrite_databases=True)
    prepare_databases(
        str(rtabmap), str(semantic), start_rtabmap=False,
        mode='mapping', overwrite_databases=True)

    assert {path: _contents(path) for path in targets} == before


def test_open_database_refuses_reset_before_either_database_changes(tmp_path):
    rtabmap = tmp_path / 'rtabmap.db'
    semantic = tmp_path / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    _make_sentinel_database(semantic)
    before = (_contents(rtabmap), _contents(semantic))
    holder = subprocess.Popen(
        [sys.executable, '-c',
         "import sys,time; stream=open(sys.argv[1], 'rb'); print('ready', flush=True); time.sleep(30)",
         str(rtabmap)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert holder.stdout.readline().strip() == 'ready'
        with pytest.raises(DatabaseLifecycleError, match='process has them open'):
            prepare_mapping_databases(str(rtabmap), str(semantic))
        assert (_contents(rtabmap), _contents(semantic)) == before
    finally:
        holder.terminate()
        holder.wait(timeout=5)


def test_read_only_parent_rejects_before_removing_other_database(tmp_path):
    read_only_parent = tmp_path / 'read-only'
    read_only_parent.mkdir()
    original_mode = read_only_parent.stat().st_mode & 0o777
    rtabmap = tmp_path / 'rtabmap.db'
    semantic = read_only_parent / 'semantic.sqlite3'
    _make_sentinel_database(rtabmap)
    before = _contents(rtabmap)
    try:
        read_only_parent.chmod(0o555)
        if os.access(read_only_parent, os.W_OK | os.X_OK):
            pytest.skip('current user can write to mode-0555 directory')
        with pytest.raises(DatabaseLifecycleError, match='not writable/searchable'):
            prepare_mapping_databases(str(rtabmap), str(semantic))
        assert _contents(rtabmap) == before
        assert not os.path.lexists(semantic)
    finally:
        read_only_parent.chmod(original_mode)
