#!/usr/bin/env bash
echo "Run test suites! "

# === pytest based test runner ===
# -Werror: warning as error
# --durations=0: show ranking of test durations
# -l (--showlocals); show local variables when test failed
pytest -n auto tests -l -Werror --durations=0
