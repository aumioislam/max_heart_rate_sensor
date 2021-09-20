.PHONY: all
all: heart_rate heart_rate_test fifo_reading_test regression_test verify_and_temp_test

heart_rate: src/ampd/ampd.o src/max30101/max30101.o src/i2c/i2c.o get_heart_rate/heart_rate.o
	$(CXX) -o $@ $^ 

heart_rate_test: src/ampd/ampd.o src/max30101/max30101.o src/i2c/i2c.o tests/heart_rate_test.o
	$(CXX) -o $@ $^ 

fifo_reading_test: src/max30101/max30101.o src/i2c/i2c.o tests/fifo_reading_test.o
	$(CXX) -o $@ $^ 

regression_test: src/ampd/ampd.o tests/regression_test.o tests/regression_test.o
	$(CXX) -o $@ $^ 

verify_and_temp_test: src/i2c/i2c.o src/max30101/max30101.o tests/verify_and_temp_test.o
	$(CXX) -o $@ $^ 

%.o: %.cpp %.hpp
	$(CXX) -c -o $@ $< 

%.o: %.c %.h
	$(CC) -c -o $@ $<

clean:
	find . -name "*.o" -type f -delete

superclean: clean
	rm -f heart_rate_test fifo_reading_test regression_test verify_and_temp_test
