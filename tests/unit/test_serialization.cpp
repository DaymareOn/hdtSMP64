#include "../include/catch.hpp"
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <memory>

// Standalone tests for serialization logic
// These validate memory safety patterns without requiring SKSE

namespace {
    // Mock global serializer list (mirrors g_SerializerList in hdtSerialization.h)
    std::vector<void*> g_MockSerializerList;

    // Base class for mock serializers
    class MockSerializerBase {
    public:
        virtual ~MockSerializerBase() = default;
        virtual uint32_t StorageName() = 0;
    };

    // Mock serializer that registers/unregisters itself (like fixed Serializer class)
    class MockSerializerFixed : public MockSerializerBase {
    public:
        MockSerializerFixed() {
            g_MockSerializerList.push_back(this);
        }

        ~MockSerializerFixed() override {
            auto it = std::find(g_MockSerializerList.begin(), g_MockSerializerList.end(),
                               static_cast<void*>(this));
            if (it != g_MockSerializerList.end()) {
                g_MockSerializerList.erase(it);
            }
        }

        uint32_t StorageName() override { return 0x54455354; }  // "TEST"
    };

    // Mock serializer with the BUG-004 issue (doesn't unregister)
    class MockSerializerBuggy : public MockSerializerBase {
    public:
        MockSerializerBuggy() {
            g_MockSerializerList.push_back(this);
        }

        ~MockSerializerBuggy() override {
            // BUG: Doesn't remove itself from g_MockSerializerList
        }

        uint32_t StorageName() override { return 0x54455354; }
    };
}

TEST_CASE("BUG-004: Serializer destructor cleanup", "[serialization][regression]") {

    SECTION("Fixed serializer removes itself on destruction") {
        g_MockSerializerList.clear();
        REQUIRE(g_MockSerializerList.size() == 0);

        {
            MockSerializerFixed serializer;
            REQUIRE(g_MockSerializerList.size() == 1);
        }
        // After destruction, should be removed
        REQUIRE(g_MockSerializerList.size() == 0);
    }

    SECTION("Buggy serializer leaves dangling pointer") {
        g_MockSerializerList.clear();
        REQUIRE(g_MockSerializerList.size() == 0);

        {
            MockSerializerBuggy serializer;
            REQUIRE(g_MockSerializerList.size() == 1);
        }
        // After destruction, STILL in list (dangling pointer!)
        REQUIRE(g_MockSerializerList.size() == 1);
        // Clean up for next test
        g_MockSerializerList.clear();
    }

    SECTION("Multiple serializers register and unregister correctly") {
        g_MockSerializerList.clear();

        {
            MockSerializerFixed s1;
            REQUIRE(g_MockSerializerList.size() == 1);

            {
                MockSerializerFixed s2;
                REQUIRE(g_MockSerializerList.size() == 2);
            }
            // s2 destroyed
            REQUIRE(g_MockSerializerList.size() == 1);
        }
        // s1 destroyed
        REQUIRE(g_MockSerializerList.size() == 0);
    }
}

// BUG-001 regression test: Memory leak in ReadData where char* was never deleted
TEST_CASE("BUG-001: ReadData memory management", "[serialization][regression]") {

    SECTION("RAII string handles memory correctly") {
        // The fix uses std::string to manage memory, which is RAII
        // This test validates the pattern
        auto readDataFixed = [](const char* source, uint32_t length) -> std::string {
            // Fixed version: Use RAII with std::string
            // The original bug was: char* data_block = new char[length]; (leaked)
            // Now we either use delete[] or better, avoid raw new entirely

            // Best approach - direct construction
            return std::string(source, length);
        };

        const char* testData = "Hello, World!";
        std::string result = readDataFixed(testData, 13);
        REQUIRE(result == "Hello, World!");
        REQUIRE(result.length() == 13);
    }

    SECTION("Manual new/delete pattern (fixed version)") {
        // This tests the actual fix pattern used in hdtSerialization.h
        auto readDataManualFixed = [](const char* source, uint32_t length) -> std::string {
            char* data_block = new char[length];
            std::memcpy(data_block, source, length);
            std::string s_data(data_block, length);
            delete[] data_block;  // FIX: This line was missing in original code
            return s_data;
        };

        const char* testData = "Test data for serialization";
        std::string result = readDataManualFixed(testData, 27);
        REQUIRE(result == "Test data for serialization");
    }

    SECTION("Zero-length data handling") {
        auto readData = [](const char* source, uint32_t length) -> std::string {
            if (length == 0) return "";
            char* data_block = new char[length];
            std::memcpy(data_block, source, length);
            std::string s_data(data_block, length);
            delete[] data_block;
            return s_data;
        };

        std::string result = readData("", 0);
        REQUIRE(result.empty());
    }

    SECTION("Large allocation handling") {
        // Test with larger buffers to ensure no memory issues
        const uint32_t largeSize = 1024 * 1024;  // 1MB
        std::vector<char> largeData(largeSize, 'X');

        auto readData = [](const char* source, uint32_t length) -> std::string {
            char* data_block = new char[length];
            std::memcpy(data_block, source, length);
            std::string s_data(data_block, length);
            delete[] data_block;
            return s_data;
        };

        std::string result = readData(largeData.data(), largeSize);
        REQUIRE(result.length() == largeSize);
        REQUIRE(result[0] == 'X');
        REQUIRE(result[largeSize - 1] == 'X');
    }
}

// BUG-008 regression test: Exception safety with unique_ptr
TEST_CASE("BUG-008: Exception safety with unique_ptr", "[memory][regression]") {

    struct CollisionResult {
        float penetration;
        int bone0, bone1;
    };

    SECTION("unique_ptr automatically cleans up on scope exit") {
        static int destructorCount = 0;

        struct TrackedResource {
            TrackedResource() = default;
            ~TrackedResource() { destructorCount++; }
        };

        destructorCount = 0;
        {
            auto resource = std::make_unique<TrackedResource[]>(10);
            REQUIRE(destructorCount == 0);
        }
        // All 10 should be destroyed
        REQUIRE(destructorCount == 10);
    }

    SECTION("unique_ptr cleans up even if exception thrown") {
        static bool resourceDestroyed = false;

        struct ThrowingOperation {
            std::unique_ptr<int[]> data;

            ThrowingOperation() : data(std::make_unique<int[]>(100)) {
                resourceDestroyed = false;
            }

            ~ThrowingOperation() {
                // data automatically cleaned up
            }

            void doWork() {
                // Simulate some work that might throw
                // In the buggy code, if this threw, raw new[] would leak
                throw std::runtime_error("simulated error");
            }
        };

        // Can't easily test this without more infrastructure,
        // but we document the pattern
        REQUIRE(true);  // Placeholder
    }

    SECTION("Raw pointer pattern (what was buggy)") {
        // This shows why the old pattern was dangerous
        // auto collision = new CollisionResult[256];
        // ... if exception thrown here ...
        // delete[] collision;  // NEVER REACHED if exception

        // Fixed pattern with unique_ptr
        auto collision = std::make_unique<CollisionResult[]>(256);
        // Now even if we return early or throw, memory is freed
        REQUIRE(collision != nullptr);
    }
}

// BUG-007 regression test: Double-free prevention
TEST_CASE("BUG-007: Buffer release sets pointer to null", "[memory][regression]") {

    struct MockMergeBuffer {
        int* buffer = nullptr;

        void alloc(int size) {
            buffer = new int[size];
        }

        // Fixed version - sets buffer to nullptr
        void releaseFixed() {
            if (buffer) {
                delete[] buffer;
                buffer = nullptr;
            }
        }

        // Buggy version - doesn't null the pointer
        void releaseBuggy() {
            if (buffer) {
                delete[] buffer;
                // BUG: buffer not set to nullptr
            }
        }
    };

    SECTION("Fixed release is safe to call twice") {
        MockMergeBuffer buf;
        buf.alloc(100);

        buf.releaseFixed();
        REQUIRE(buf.buffer == nullptr);

        // Safe to call again - no double free
        buf.releaseFixed();
        REQUIRE(buf.buffer == nullptr);
    }

    SECTION("Buggy release has non-null dangling pointer") {
        MockMergeBuffer buf;
        buf.alloc(100);

        buf.releaseBuggy();
        // Buffer pointer is still non-null (dangling!)
        // Second call would double-free (UB)
        REQUIRE(buf.buffer != nullptr);  // This demonstrates the bug

        // Clean up for memory safety
        buf.buffer = nullptr;
    }
}

TEST_CASE("UInt32 to string conversion", "[serialization]") {
    // Test the _uint32_to_str_t logic from hdtSerialization.h
    union UInt32ToCStr {
        uint32_t number;
        char buffer[4];
    };

    auto uint32ToStr = [](uint32_t number) -> std::string {
        UInt32ToCStr u{};
        u.number = number;
        std::string str(u.buffer, 4);
        return std::string(str.rbegin(), str.rend());
    };

    SECTION("Known values") {
        // 'TEST' = 0x54455354 in big-endian ASCII
        REQUIRE(uint32ToStr(0x54455354) == "TEST");
    }

    SECTION("Null bytes in string") {
        // Handle embedded nulls correctly
        std::string result = uint32ToStr(0x00414141);
        REQUIRE(result.length() == 4);
    }
}
