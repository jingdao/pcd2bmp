SRC = pcd2bmp.c
INC = pcd2bmp.h
OUT = pcd2bmp

all: $(SRC) $(INC)
	gcc -o $(OUT) $(SRC) -lm 

clean:
	rm $(OUT)
