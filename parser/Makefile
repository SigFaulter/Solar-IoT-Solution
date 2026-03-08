CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Iinclude
LDFLAGS  :=

BUILD  := build
TARGET := $(BUILD)/mppt_parser

SRCS := src/main.cpp        \
        src/space_parser.cpp \
        src/eeprom_parser.cpp \
        src/printer.cpp      \
        src/json_builder.cpp

OBJS := $(patsubst src/%.cpp, $(BUILD)/%.o, $(SRCS))

JSON_HPP := include/json.hpp
JSON_URL := https://github.com/nlohmann/json/releases/download/v3.12.0/json.hpp

$(JSON_HPP):
	curl -L $(JSON_URL) -o $@

$(BUILD)/%.o: src/%.cpp $(JSON_HPP) | $(BUILD)

.PHONY: all release debug clean

all: release

release: CXXFLAGS += -O2 -DNDEBUG
release: LDFLAGS  += -s
release: $(TARGET)

debug: CXXFLAGS += -O0 -g -fsanitize=address,undefined
debug: LDFLAGS  += -fsanitize=address,undefined
debug: $(TARGET)

$(TARGET): $(OBJS) | $(BUILD)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ -o $@

$(BUILD)/%.o: src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
