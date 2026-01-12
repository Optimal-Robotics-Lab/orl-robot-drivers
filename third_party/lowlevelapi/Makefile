# Build example using the .c files in this directory and the static libartl.a
lowlevelapi_example: lowlevelapi_example.c lowlevelapi.c libartl/libartl.a
	gcc $^ -o $@

# Build the static ARTL library
libartl/libartl.a:
	$(MAKE) -C libartl libartl.a

# Clean outputs from both this directory and libartl
clean:
	rm -f lowlevelapi_example
	$(MAKE) -C libartl clean

.PHONY: clean
