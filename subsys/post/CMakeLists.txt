# Copyright (c) 2026 Aerlync Labs Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(
  post_core.c
)

zephyr_library_sources_ifdef(CONFIG_POST_USERSPACE
  post_syscalls.c
)

zephyr_library_sources_ifdef(CONFIG_POST_SHELL
  post_shell.c
)

# Built-in tests
zephyr_library_sources_ifdef(CONFIG_POST_CPU_TESTS
  tests/cpu/cpu_tests.c
)

zephyr_library_sources_ifdef(CONFIG_POST_RAM_TESTS
  tests/ram/ram_tests.c
)

zephyr_library_sources_ifdef(CONFIG_POST_STACK_TESTS
  tests/stack/stack_tests.c
)

zephyr_library_sources_ifdef(CONFIG_POST_FLASH_TESTS
  tests/flash/flash_tests.c
)

zephyr_library_sources_ifdef(CONFIG_POST_CLOCK_TESTS
  tests/clock/clock_tests.c
)
