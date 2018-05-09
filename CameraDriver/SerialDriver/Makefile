TARGET=serial_test2

all: main.c
	gcc main.c serial.c -o $(TARGET)

run: all
	$(TARGET)

clean:
	rm *.exe

sender: sender.c serial.c 
	gcc sender.c serial.c -o sender_test

reciever: reciever.c serial.c 
	gcc reciever.c serial.c -o reciever_test