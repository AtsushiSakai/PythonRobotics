#!/usr/bin/env bash
echo "Run test suites! "
# tests: include unittest based tests
# -Werror: warning as error
# --durations=0: show ranking of test durations
pytest tests -Werror --durations=0
