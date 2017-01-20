
TARGET	?= app

$(TARGET):$(TARGET:=.o)
	arm-linux-gcc -o $@ $^

%.o:%.c
	arm-linux-gcc -c -o $@ $<

clean:
	rm -rf $(TARGET) $(TARGET:=.o)
