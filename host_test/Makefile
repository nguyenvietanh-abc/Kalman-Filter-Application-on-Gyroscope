# Bien dich va chay bo kiem chung Kalman tren PC - khong can ESP32.
#
# Diem quan trong: cac file trong ../main/ o day la CHINH nhung file duoc
# firmware bien dich, khong phai ban copy. Bat duoc loi o day la bat duoc loi
# tren thiet bi that.

CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -g -Wall -Wextra -Wshadow -Wpointer-arith -Werror
CFLAGS  += -I../main
LDLIBS  := -lm

CORE_SRC := ../main/kalman.c ../main/ahrs.c ../main/imu_calib.c
TEST_SRC := imu_sim.c test_kalman.c
SRC      := $(CORE_SRC) $(TEST_SRC) test_convention.c
OBJ      := $(notdir $(SRC:.c=.o))
BIN      := test_kalman
BIN2     := test_convention

VPATH := ../main

.PHONY: all run clean
all: $(BIN) $(BIN2)

$(BIN): kalman.o ahrs.o imu_calib.o imu_sim.o test_kalman.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

# Neo quy uoc quay vao vat ly - chi can ahrs.c (dung ahrs_accel_angles).
$(BIN2): kalman.o ahrs.o test_convention.o
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

run: $(BIN) $(BIN2)
	./$(BIN2)
	./$(BIN)

clean:
	rm -f $(OBJ) $(BIN) $(BIN2) out.csv out_uncal.csv out_bias.csv *.png run.log

# Phu thuoc header - de sua header la bien dich lai dung phan can thiet
kalman.o:      ../main/kalman.h
ahrs.o:        ../main/ahrs.h ../main/kalman.h
imu_calib.o:   ../main/imu_calib.h ../main/ahrs.h
imu_sim.o:     imu_sim.h ../main/ahrs.h
test_kalman.o: imu_sim.h ../main/ahrs.h ../main/kalman.h ../main/imu_calib.h
test_convention.o: ../main/ahrs.h ../main/kalman.h
