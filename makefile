CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -O3 -Iinclude
DEBUGFLAGS = -g -O0

SRCDIR = src
TESTDIR = tests
OBJDIR = build
SOURCES = $(shell find $(SRCDIR) -name "*.cpp")
TEST_SOURCES = $(shell find $(TESTDIR) -name "*.cpp") $(filter-out $(SRCDIR)/main.cpp, $(SOURCES))
OBJECTS = $(SOURCES:%.cpp=$(OBJDIR)/%.o)
TARGET = valerie
TEST_TARGET = test_runner

.PHONY: all clean debug test

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TEST_TARGET)
	./$(TEST_TARGET)

$(TEST_TARGET): $(TEST_SOURCES)
	$(CXX) $(CXXFLAGS) $(TEST_SOURCES) -o $@

debug: CXXFLAGS += $(DEBUGFLAGS)
debug: $(TARGET)

clean:
	rm -rf $(OBJDIR) $(TARGET) $(TEST_TARGET)
