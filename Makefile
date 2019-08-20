COMPILE_OPTS = -MMD -std=gnu++17 -O2 -g3 -Wall $(INCLUDE_DIRS)

CC = g++ # This is the main compiler
CXX = $(CC)
SRCDIR = src
BUILDDIR = build/x86
TARGET = bin/x86/runner
TARGET_TEST = bin/x86/tester
 
SRCEXT = cpp
OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(wildcard $(SRCDIR)/*.cpp))
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
DEPS = $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.d,$(wildcard $(SRCDIR)/*.cpp))
CFLAGS = $(COMPILE_OPTS) # -Wall
CXXFLAGS = $(COMPILE_OPTS) # -Wall
INC = -I include

$(TARGET): $(OBJECTS)
	@echo " Linking..."
	@mkdir -p $(dir $(TARGET))
	@echo " $(CC) $^ -o $(TARGET) $(LIB)"; $(CC) $^ -o $(TARGET) $(LIB)

$(BUILDDIR)/tester.o: test/tester.cpp
	@mkdir -p $(BUILDDIR) 
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR) 
	@echo " $(CC) $(CFLAGS) $(INC) -c -o $@ $<"; $(CC) $(CFLAGS) $(INC) -c -o $@ $<

run: $(TARGET)
	@$(TARGET)

runtest: $(TARGET_TEST)
	@$(TARGET_TEST)

clean:
	@echo " Cleaning..."; 
	@echo " $(RM) -r $(BUILDDIR) $(TARGET)"; $(RM) -r $(BUILDDIR) $(TARGET)

# Tests
test: $(TARGET_TEST)

$(TARGET_TEST): $(OBJECTS) $(BUILDDIR)/tester.o
	$(CC) $(CFLAGS) $^ $(INC) $(LIB) -o $(TARGET_TEST)

.PHONY: clean

-include $(DEPS)
