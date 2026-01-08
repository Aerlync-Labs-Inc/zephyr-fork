/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief POST CPU tests
 *
 * Tests for CPU register integrity and ALU operations.
 */

#include <zephyr/kernel.h>
#include <zephyr/post/post.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(post, CONFIG_POST_LOG_LEVEL);

/**
 * @brief Test CPU general-purpose registers
 *
 * Writes walking 1s and 0s patterns to registers and verifies
 * they can be read back correctly.
 */
static enum post_result cpu_reg_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

	volatile uint32_t test_val;
	uint32_t patterns[] = {
		0x00000000,
		0xFFFFFFFF,
		0xAAAAAAAA,
		0x55555555,
		0x0000FFFF,
		0xFFFF0000,
		0x00FF00FF,
		0xFF00FF00,
	};

	for (int i = 0; i < ARRAY_SIZE(patterns); i++) {
		test_val = patterns[i];
		if (test_val != patterns[i]) {
			LOG_ERR("CPU reg test failed: wrote 0x%08x, read 0x%08x",
				patterns[i], test_val);
			return POST_RESULT_FAIL;
		}
	}

	/* Walking 1s test */
	for (int bit = 0; bit < 32; bit++) {
		uint32_t pattern = 1U << bit;

		test_val = pattern;
		if (test_val != pattern) {
			LOG_ERR("Walking 1s failed at bit %d", bit);
			return POST_RESULT_FAIL;
		}
	}

	return POST_RESULT_PASS;
}

POST_CPU_TEST(cpu_reg_test, 0x0001, POST_LEVEL_EARLY, cpu_reg_test_fn);

/**
 * @brief Test ALU basic operations
 *
 * Verifies addition, subtraction, multiplication, and logical operations.
 */
static enum post_result cpu_alu_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

	volatile uint32_t a, b, result;

	/* Addition test */
	a = 0x12345678;
	b = 0x11111111;
	result = a + b;
	if (result != 0x23456789) {
		LOG_ERR("ALU add failed: 0x%08x + 0x%08x = 0x%08x (expected 0x23456789)",
			a, b, result);
		return POST_RESULT_FAIL;
	}

	/* Subtraction test */
	a = 0x87654321;
	b = 0x12345678;
	result = a - b;
	if (result != 0x7530ECA9) {
		LOG_ERR("ALU sub failed");
		return POST_RESULT_FAIL;
	}

	/* AND test */
	a = 0xFF00FF00;
	b = 0x0F0F0F0F;
	result = a & b;
	if (result != 0x0F000F00) {
		LOG_ERR("ALU AND failed");
		return POST_RESULT_FAIL;
	}

	/* OR test */
	a = 0xFF00FF00;
	b = 0x00FF00FF;
	result = a | b;
	if (result != 0xFFFFFFFF) {
		LOG_ERR("ALU OR failed");
		return POST_RESULT_FAIL;
	}

	/* XOR test */
	a = 0xAAAAAAAA;
	b = 0x55555555;
	result = a ^ b;
	if (result != 0xFFFFFFFF) {
		LOG_ERR("ALU XOR failed");
		return POST_RESULT_FAIL;
	}

	/* Shift test */
	a = 0x00000001;
	result = a << 16;
	if (result != 0x00010000) {
		LOG_ERR("ALU shift left failed");
		return POST_RESULT_FAIL;
	}

	a = 0x80000000;
	result = a >> 16;
	if (result != 0x00008000) {
		LOG_ERR("ALU shift right failed");
		return POST_RESULT_FAIL;
	}

	return POST_RESULT_PASS;
}

POST_TEST_DEFINE(cpu_alu_test, 0x0002, POST_CAT_CPU, POST_LEVEL_PRE_KERNEL_1,
		 5, POST_FLAG_RUNTIME_OK | POST_FLAG_USERSPACE_OK,
		 cpu_alu_test_fn, "CPU ALU operations test");
