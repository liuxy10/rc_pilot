import re
import numpy as np
import matplotlib.pyplot as plt

# parse txt data file to numpy array 
# input: name of data file
# output: time: (n,) array of timestamp
#         accel_gyro: (n, 6) array, first 3 columns are Accel XYZ(m/s^2) 
#                                   last 3 columns are  Gyro XYZ (deg/s)
def parse_data_to_nparray(file_name):
    with open(file_name) as f:
        lines = f.readlines()
    count = 0
    time = []
    accel_gyro = []
    for line in lines:
        count += 1
        # skip first lines with titles
        if count >= 6:
            if count % 2 == 0:
                # only time
                time.append(float(line)/1e6)
            else: 
                split_line = re.split(' |\t|\n|\|', line)
                data_arr = []
                for char in split_line:
                    if len(char) > 0:
                        data_arr.append(float(char))
                assert len(data_arr)==6
                accel_gyro.append(data_arr)
    return np.array(time), np.array(accel_gyro)

def find_freq(t, f):
    L = t.shape[0]
    Fs = 1/((t[-1]-t[0])/(L-1))
    #print("Sampling rate: {}".format(Fs))
    T = 1/Fs 
    Y = np.fft.fft(f)
    P2 = np.abs(Y/L)
    P1 = P2[0: L//2 + 1]
    P1[1: -1] = P1[1: -1] * 2
    f = Fs * np.arange(L//2+1)/L
    plt.figure()
    plt.plot(f, P1)
    plt.title("FFT of angular velocity signal")
    plt.xlabel("Frequency")
    plt.ylabel("Amplitude")
    freq = f[np.argmax(P1)]
    print("Frequency: {} ".format(freq))
    print("Period: {} ".format(1/freq))
    return  1/freq

def main():
    time_imu = []
    gyro_accel = []
    periods = []
    
    time_imu_x, gyro_accel_x= parse_data_to_nparray('data/data_imu_x.txt')
    time_imu_y, gyro_accel_y= parse_data_to_nparray('data/data_imu_y.txt')
    time_imu_z, gyro_accel_z= parse_data_to_nparray('data/data_imu_z.txt')
    time_imu_all = [time_imu_x, time_imu_y, time_imu_z]
    gyro_accel_all = [gyro_accel_x, gyro_accel_y, gyro_accel_z]
    
    m0 = 1.037
    g = 9.8
    d = 0.33
    L = 2.05
    
    # plot angular  largest varying angular velocity 
    for time_imu, gyro_accel, dim, col in zip(time_imu_all, gyro_accel_all, ['x', 'y', 'z'], [3, 4, 5]):
        plt.figure()
        plt.plot(time_imu, gyro_accel[:, col])
        plt.title("Bifilar Pendulum Around IMU-{} axis, w{} - time Plot".format(dim, dim))
        plt.xlabel("Time(s)")
        plt.ylabel("Angular Velocity (deg/s)")
        T0 = find_freq(time_imu, gyro_accel[:, col])
        J = m0 * g * d**2 /(16 * np.pi**2 * L) * T0**2
        print("Moment of Inertia around imu {} is {}\n".format(dim, J))
    plt.show()

    
if __name__ == '__main__':
    main()