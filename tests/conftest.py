"""pytest 共通設定。"""
import pytest

# pytest はテスト0件のとき終了コード 5 を返す。
# STEP 1（テスト未作成）の段階でも「成功」と扱うため 0 に読み替える。
NO_TESTS_COLLECTED = pytest.ExitCode.NO_TESTS_COLLECTED


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    """テスト0件を成功扱いにする。"""
    if exitstatus == NO_TESTS_COLLECTED:
        session.exitstatus = pytest.ExitCode.OK
