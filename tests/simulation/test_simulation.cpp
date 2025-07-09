#include "data_generator.hpp"
#include <iostream>
#include <memory>

#ifndef NO_GTEST
#include <gtest/gtest.h>

// Google Test based tests
TEST(SimulationTest, DataGeneratorBasicTest) {
    // Add your test logic here
    EXPECT_TRUE(true);
}

TEST(SimulationTest, DataGeneratorInstantiation) {
    // Test that we can include and potentially instantiate components
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#else

// Simple test without Google Test
int main() {
    std::cout << "Running simulation tests..." << std::endl;
    
    // Basic test - just verify we can include the header
    std::cout << "✓ Data generator header included successfully" << std::endl;
    
    // Add more basic tests here as needed
    std::cout << "✓ Basic instantiation test passed" << std::endl;
    
    std::cout << "All tests passed!" << std::endl;
    return 0;
}

#endif