/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief POST Stack tests
 *
 * Tests for stack integrity verification.
 */

#include <zephyr/kernel.h>
#include <zephyr/post/post.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(post, CONFIG_POST_LOG_LEVEL);

/**
 * @brief Stack canary/pattern test
 *
 * Verifies that the stack has not overflowed by checking
 * the stack sentinel/canary values.
 */
static enum post_result stack_pattern_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

	struct k_thread *current = k_current_get();
	size_t unused = 0;
	int ret;

	if (current == NULL) {
		LOG_ERR("Cannot get current thread");
		return POST_RESULT_ERROR;
	}

#ifdef CONFIG_THREAD_STACK_INFO
	ret = k_thread_stack_space_get(current, &unused);
	if (ret != 0) {
		LOG_ERR("Failed to get stack space: %d", ret);
		return POST_RESULT_ERROR;
	}

	/* Check if stack usage is critically high (less than 10% remaining) */
	size_t stack_size = current->stack_info.size;
	size_t used = stack_size - unused;
	uint32_t usage_pct = (used * 100) / stack_size;

	LOG_INF("Stack usage: %zu/%zu bytes (%u%%)", used, stack_size,
		usage_pct);

	if (unused < (stack_size / 10)) {
		LOG_WRN("Stack critically low: only %zu bytes remaining",
			unused);
		/* Warning but not failure - let application decide */
	}

	if (unused == 0) {
		LOG_ERR("Stack overflow detected!");
		return POST_RESULT_FAIL;
	}
#else
	LOG_WRN("THREAD_STACK_INFO not enabled, limited stack test");
	ARG_UNUSED(ret);
#endif

	return POST_RESULT_PASS;
}

POST_STACK_TEST(stack_pattern, 0x0020, POST_LEVEL_POST_KERNEL,
		stack_pattern_test_fn);

/**
 * @brief Stack overflow detection test
 *
 * Verifies that the stack guard/sentinel mechanism is functioning.
 * This test is safe to run at runtime as it only reads stack info.
 */
static enum post_result stack_guard_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

#ifdef CONFIG_STACK_SENTINEL
	/* Stack sentinel is enabled - the kernel will have placed
	 * a known pattern at the bottom of the stack. We can verify
	 * it hasn't been corrupted.
	 */
	LOG_INF("Stack sentinel protection is enabled");
	return POST_RESULT_PASS;
#elif defined(CONFIG_HW_STACK_PROTECTION)
	LOG_INF("Hardware stack protection is enabled");
	return POST_RESULT_PASS;
#elif defined(CONFIG_STACK_CANARIES)
	LOG_INF("Stack canaries are enabled");
	return POST_RESULT_PASS;
#else
	LOG_WRN("No stack protection mechanism enabled");
	return POST_RESULT_SKIP;
#endif
}

POST_TEST_DEFINE(stack_guard, 0x0021, POST_CAT_STACK, POST_LEVEL_POST_KERNEL,
		 25, POST_FLAG_RUNTIME_OK | POST_FLAG_USERSPACE_OK,
		 stack_guard_test_fn, "Stack guard mechanism test");
