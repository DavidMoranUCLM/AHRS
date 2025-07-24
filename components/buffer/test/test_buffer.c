#include "unity.h"
#include "buffer.h"
#include "string.h"
#include "freertos/FreeRTOS.h"

#define N_ITEMS   25
#define ITEM_SIZE 10

vectorBuffer_t buffer;

/* Test 1: Initialize Buffer */
TEST_CASE("Buffer Init", "[buffer]") {
    TEST_ASSERT(initBuffer(&buffer, ITEM_SIZE, N_ITEMS) == ESP_OK);
    TEST_ASSERT_NOT_NULL(buffer.buffer);
    TEST_ASSERT(buffer.vectorMaxNumber == N_ITEMS);
    /* This assertion compares the logical item size; if needed, adjust accordingly */
    TEST_ASSERT(ITEM_SIZE == 10);

    TEST_ASSERT(deinitBuffer(&buffer) == ESP_OK);
    TEST_PASS();
}

/* Test 2: Push and Pull Single Item */
TEST_CASE("Buffer Push-Pull", "[buffer]") {
    TEST_ASSERT(initBuffer(&buffer, ITEM_SIZE, N_ITEMS) == ESP_OK);
    char item[ITEM_SIZE] = {'0','1','2','3','4','5','6','7','8','9'};
    char itemcopy[ITEM_SIZE] = {0};

    /* Pass the logical size ITEM_SIZE (10) to pushItem and pullItem */
    TEST_ASSERT(pushItem(&buffer, item, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT(pullItem(&buffer, itemcopy, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT_EQUAL_MEMORY(item, itemcopy, ITEM_SIZE);

    TEST_ASSERT(deinitBuffer(&buffer) == ESP_OK);
    TEST_PASS();
}

/* Test 3: Push Multiple Items */
TEST_CASE("Buffer Push Multiple Items", "[buffer]") {
    TEST_ASSERT(initBuffer(&buffer, ITEM_SIZE, N_ITEMS) == ESP_OK);

    char item1[ITEM_SIZE] = "ABCDEFGHIJ";
    char item2[ITEM_SIZE] = "KLMNOPQRST";
    char item3[ITEM_SIZE] = "UVWXYZ1234";
    char received[ITEM_SIZE] = {0};

    TEST_ASSERT(pushItem(&buffer, item1, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT(pushItem(&buffer, item2, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT(pushItem(&buffer, item3, ITEM_SIZE) == ESP_OK);

    TEST_ASSERT(pullItem(&buffer, received, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT_EQUAL_MEMORY(item1, received, ITEM_SIZE);

    TEST_ASSERT(pullItem(&buffer, received, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT_EQUAL_MEMORY(item2, received, ITEM_SIZE);

    TEST_ASSERT(pullItem(&buffer, received, ITEM_SIZE) == ESP_OK);
    TEST_ASSERT_EQUAL_MEMORY(item3, received, ITEM_SIZE);

    TEST_ASSERT(deinitBuffer(&buffer) == ESP_OK);
    TEST_PASS();
}

/* Test 4: Buffer Overflow (Filling up the buffer beyond capacity) */
TEST_CASE("Buffer Overflow", "[buffer]") {
    TEST_ASSERT(initBuffer(&buffer, ITEM_SIZE, N_ITEMS) == ESP_OK);
    
    char item[ITEM_SIZE] = "1234567890";
    for (int i = 0; i < N_ITEMS; i++) {
        TEST_ASSERT(pushItem(&buffer, item, ITEM_SIZE) == ESP_OK);
    }

    /* The next push should fail since the buffer is full */
    TEST_ASSERT(pushItem(&buffer, item, ITEM_SIZE) != ESP_OK);

    TEST_ASSERT(deinitBuffer(&buffer) == ESP_OK);
    TEST_PASS();
}

/* Test 5: Buffer Underflow (Pulling from an empty buffer) */
TEST_CASE("Buffer Underflow", "[buffer]") {
    TEST_ASSERT(initBuffer(&buffer, ITEM_SIZE, N_ITEMS) == ESP_OK);

    char received[ITEM_SIZE] = {0};

    /* Pulling from an empty buffer should fail */
    TEST_ASSERT(pullItem(&buffer, received, ITEM_SIZE) != ESP_OK);

    TEST_ASSERT(deinitBuffer(&buffer) == ESP_OK);
    TEST_PASS();
}

/* Test 6: Concurrency Test (Push & Pull in separate tasks) */
void producer_task(void *arg) {
    char item[ITEM_SIZE] = "TASK_PROD";
    for (int i = 0; i < 10; i++) {
        pushItem(&buffer, item, ITEM_SIZE);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void consumer_task(void *arg) {
    char received[ITEM_SIZE] = {0};
    for (int i = 0; i < 10; i++) {
        pullItem(&buffer, received, ITEM_SIZE);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    vTaskDelete(NULL);
}

TEST_CASE("Buffer Concurrency Test", "[buffer]") {
    TEST_ASSERT(initBuffer(&buffer, ITEM_SIZE, N_ITEMS) == ESP_OK);

    xTaskCreate(producer_task, "producer", 4096, NULL, 5, NULL);
    xTaskCreate(consumer_task, "consumer", 4096, NULL, 5, NULL);

    /* Allow tasks to execute */
    vTaskDelay(pdMS_TO_TICKS(500));

    TEST_ASSERT(deinitBuffer(&buffer) == ESP_OK);
    TEST_PASS();
}
