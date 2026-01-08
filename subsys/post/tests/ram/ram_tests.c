/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief POST RAM tests
 *
 * Memory integrity tests using various patterns.
 * WARNING: These tests are destructive and should only run at boot.
 */

#include <zephyr/kernel.h>
#include <zephyr/post/post.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(post, CONFIG_POST_LOG_LEVEL);

/* Test buffer - placed in a known RAM region */
#define RAM_TEST_SIZE 256
static volatile uint8_t ram_test_buffer[RAM_TEST_SIZE] __aligned(4);

/**
 * @brief March C- RAM test
 *
 * A well-known memory test algorithm that detects:
 * - Stuck-at faults
 * - Transition faults
 * - Coupling faults
 *
 * Algorithm:
 * 1. Write 0 to all cells ascending
 * 2. Read 0, write 1 ascending
 * 3. Read 1, write 0 descending
 * 4. Read 0, write 1 descending
 * 5. Read 1, write 0 ascending
 * 6. Read 0 ascending
 */
static enum post_result ram_march_c_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

	volatile uint8_t *buf = ram_test_buffer;
	size_t size = RAM_TEST_SIZE;
	size_t i;

	/* Step 1: Write 0 ascending */
	for (i = 0; i < size; i++) {
		buf[i] = 0x00;
	}

	/* Step 2: Read 0, write 1 ascending */
	for (i = 0; i < size; i++) {
		if (buf[i] != 0x00) {
			LOG_ERR("March C- step 2 failed at offset %zu", i);
			return POST_RESULT_FAIL;
		}
		buf[i] = 0xFF;
	}

	/* Step 3: Read 1, write 0 descending */
	for (i = size; i > 0; i--) {
		if (buf[i - 1] != 0xFF) {
			LOG_ERR("March C- step 3 failed at offset %zu", i - 1);
			return POST_RESULT_FAIL;
		}
		buf[i - 1] = 0x00;
	}

	/* Step 4: Read 0, write 1 descending */
	for (i = size; i > 0; i--) {
		if (buf[i - 1] != 0x00) {
			LOG_ERR("March C- step 4 failed at offset %zu", i - 1);
			return POST_RESULT_FAIL;
		}
		buf[i - 1] = 0xFF;
	}

	/* Step 5: Read 1, write 0 ascending */
	for (i = 0; i < size; i++) {
		if (buf[i] != 0xFF) {
			LOG_ERR("March C- step 5 failed at offset %zu", i);
			return POST_RESULT_FAIL;
		}
		buf[i] = 0x00;
	}

	/* Step 6: Read 0 ascending */
	for (i = 0; i < size; i++) {
		if (buf[i] != 0x00) {
			LOG_ERR("March C- step 6 failed at offset %zu", i);
			return POST_RESULT_FAIL;
		}
	}

	return POST_RESULT_PASS;
}

POST_RAM_TEST(ram_march_c, 0x0010, POST_LEVEL_PRE_KERNEL_1, ram_march_c_test_fn);

/**
 * @brief Checkerboard RAM test
 *
 * Writes alternating 0x55/0xAA patterns and verifies.
 */
static enum post_result ram_checkerboard_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

	volatile uint8_t *buf = ram_test_buffer;
	size_t size = RAM_TEST_SIZE;
	size_t i;

	/* Write checkerboard pattern */
	for (i = 0; i < size; i++) {
		buf[i] = (i & 1) ? 0xAA : 0x55;
	}

	/* Verify */
	for (i = 0; i < size; i++) {
		uint8_t expected = (i & 1) ? 0xAA : 0x55;

		if (buf[i] != expected) {
			LOG_ERR("Checkerboard failed at offset %zu: "
				"expected 0x%02x, got 0x%02x",
				i, expected, buf[i]);
			return POST_RESULT_FAIL;
		}
	}

	/* Write inverse pattern */
	for (i = 0; i < size; i++) {
		buf[i] = (i & 1) ? 0x55 : 0xAA;
	}

	/* Verify inverse */
	for (i = 0; i < size; i++) {
		uint8_t expected = (i & 1) ? 0x55 : 0xAA;

		if (buf[i] != expected) {
			LOG_ERR("Checkerboard inverse failed at offset %zu",
				i);
			return POST_RESULT_FAIL;
		}
	}

	return POST_RESULT_PASS;
}

POST_RAM_TEST(ram_checkerboard, 0x0011, POST_LEVEL_PRE_KERNEL_2,
	      ram_checkerboard_test_fn);
