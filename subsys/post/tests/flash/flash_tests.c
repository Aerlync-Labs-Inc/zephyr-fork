/*
 * Copyright (c) 2026 Aerlync Labs Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief POST Flash/ROM tests
 *
 * Tests for Flash/ROM integrity verification.
 */

#include <zephyr/kernel.h>
#include <zephyr/post/post.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(post, CONFIG_POST_LOG_LEVEL);

/* These symbols are defined by the linker script */
extern char __rom_region_start[];
extern char __rom_region_end[];

/* Golden CRC value - this would be stored in flash during production */
#ifndef CONFIG_POST_FLASH_GOLDEN_CRC
#define CONFIG_POST_FLASH_GOLDEN_CRC 0
#endif

/**
 * @brief Flash CRC verification test
 *
 * Calculates CRC32 of the code region and compares against
 * a stored golden value.
 *
 * Note: For this test to be meaningful, the golden CRC must be
 * calculated and stored during production programming.
 */
static enum post_result flash_crc_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

#if CONFIG_POST_FLASH_GOLDEN_CRC != 0
	const uint8_t *start = (const uint8_t *)__rom_region_start;
	const uint8_t *end = (const uint8_t *)__rom_region_end;
	size_t size = end - start;
	uint32_t calculated_crc;

	if (size == 0) {
		LOG_ERR("ROM region size is 0");
		return POST_RESULT_ERROR;
	}

	LOG_INF("Calculating CRC32 of ROM region (%zu bytes)", size);

	calculated_crc = crc32_ieee(start, size);

	LOG_INF("Calculated CRC: 0x%08x, Golden CRC: 0x%08x",
		calculated_crc, CONFIG_POST_FLASH_GOLDEN_CRC);

	if (calculated_crc != CONFIG_POST_FLASH_GOLDEN_CRC) {
		LOG_ERR("Flash CRC mismatch!");
		return POST_RESULT_FAIL;
	}

	return POST_RESULT_PASS;
#else
	LOG_WRN("Flash golden CRC not configured, skipping test");
	return POST_RESULT_SKIP;
#endif
}

POST_FLASH_TEST(flash_crc, 0x0030, POST_LEVEL_APPLICATION, flash_crc_test_fn);

/**
 * @brief Flash read test
 *
 * Basic test to verify flash can be read without errors.
 * Reads a few bytes from the code region and verifies no faults occur.
 */
static enum post_result flash_read_test_fn(const struct post_context *ctx)
{
	ARG_UNUSED(ctx);

	const volatile uint32_t *code = (const volatile uint32_t *)__rom_region_start;
	uint32_t checksum = 0;

	/* Read first 64 words of code region */
	for (int i = 0; i < 64; i++) {
		checksum += code[i];
	}

	/* Just verify we can read without faulting */
	LOG_INF("Flash read test checksum: 0x%08x", checksum);

	return POST_RESULT_PASS;
}

POST_TEST_DEFINE(flash_read, 0x0031, POST_CAT_FLASH, POST_LEVEL_POST_KERNEL,
		 35, POST_FLAG_RUNTIME_OK | POST_FLAG_USERSPACE_OK,
		 flash_read_test_fn, "Flash read verification test");
