TARGET   := bin/crying_app

CC       := gcc
CSTD     := -std=c17
WARN     := -Wall -Wextra
CFLAGS   := -D_GNU_SOURCE

APP_MAIN := crying_module.c

INC_DIRS := . ../library

LIB_SRCS := $(wildcard ../library/*.c)
SRCS     := $(APP_MAIN) $(LIB_SRCS)

OBJS     := $(patsubst %.c,build/obj/%.o,$(SRCS))
DEPS     := $(OBJS:.o=.d)

LDFLAGS  :=
LDLIBS   := -lm -lpthread

.PHONY: all
all: $(TARGET)

$(TARGET): $(OBJS)
	@mkdir -p $(dir $@)
	$(CC) $(OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

build/obj/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CFLAGS) -MMD -MP \
		$(addprefix -I,$(INC_DIRS)) \
		-c $< -o $@

-include $(DEPS)

.PHONY: clean
clean:
	rm -rf build
	rm -rf bin