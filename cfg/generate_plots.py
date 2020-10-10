import numpy as np
import matplotlib.pyplot as plt
import csv
import bisect

def readInterpolatedValues(files, times):
    detrois = []
    volaccs = []
    for filename in files:
        r0, r1, r4 = [], [], []
        with open(filename,'r') as csvfile:
            csv_reader = csv.reader(csvfile, delimiter=',')
            next(csv_reader)
            for row in csv_reader:
                r0.append(float(row[0]))
                r1.append(int(row[1]))
                r4.append(float(row[4]))
    
        # interpolate data to whole seconds for easier averaging
        r1_n, r4_n = [], []
        ci = 0
        for t in times:
            if ci < len(times):
                ci = bisect.bisect_left(r0, t, ci)
            if ci >= len(times): # end of times reached; keep last value
                r1_n.append(r1[-1])
                r4_n.append(r4[-1])
            elif ci == 0: # if t smaller than first value, keep first
                r1_n.append(r1[0])
                r4_n.append(r4[0])
            else:
                t1 = r0[ci - 1]
                t2 = r0[ci]
                dr = r1[ci - 1] * (t - t1) / (t2 - t1) + r1[ci] * (t2 - t) / (t2 - t1)
                va = r4[ci - 1] * (t - t1) / (t2 - t1) + r4[ci] * (t2 - t) / (t2 - t1)
                r1_n.append(int(dr)) 
                r4_n.append(va)    
    
        detrois.append(r1_n)
        volaccs.append(r4_n)

    return detrois, volaccs


auto_files = ['automatic/planner_results_0.csv', 'automatic/planner_results_1.csv', 'automatic/planner_results_2.csv', 'automatic/planner_results_3.csv', 'automatic/planner_results_4.csv', 'automatic/planner_results_5.csv', 'automatic/planner_results_6.csv', 'automatic/planner_results_7.csv', 'automatic/planner_results_8.csv', 'automatic/planner_results_9.csv']

contour_files = ['contours/planner_results_0.csv', 'contours/planner_results_1.csv', 'contours/planner_results_2.csv', 'contours/planner_results_3.csv', 'contours/planner_results_4.csv', 'contours/planner_results_5.csv', 'contours/planner_results_6.csv', 'contours/planner_results_7.csv']

times = range(181)

detrois_auto, volaccs_auto = readInterpolatedValues(auto_files, times)
detrois_contours, volaccs_contours = readInterpolatedValues(contour_files, times)

detroi_auto_avg = np.average(detrois_auto, axis=0)
detroi_contours_avg = np.average(detrois_contours, axis=0)

volacc_auto_avg = np.average(volaccs_auto, axis=0)
volacc_contours_avg = np.average(volaccs_contours, axis=0)

plt.figure(1)

for detroi in detrois_auto:
    plt.plot(times, detroi, 'C1--', alpha=0.2)
for detroi in detrois_contours:
    plt.plot(times, detroi, 'C2--', alpha=0.2)

plt.plot(times, detroi_auto_avg, 'C1', linewidth=3.0)
plt.plot(times, detroi_contours_avg, 'C2', linewidth=3.0)

plt.xlabel('Time (s)')
plt.ylabel('Detected ROIs')
plt.title('Detected ROIs')
plt.legend()
plt.show()

plt.figure(2)

for volacc in volaccs_auto:
    plt.plot(times, volacc, 'C1--', alpha=0.2)
for volacc in volaccs_contours:
    plt.plot(times, volacc, 'C2--', alpha=0.2)

plt.plot(times, volacc_auto_avg, 'C1', linewidth=3.0)
plt.plot(times, volacc_contours_avg, 'C2', linewidth=3.0)

plt.xlabel('Time (s)')
plt.ylabel('Volume accuracy')
plt.title('Volume accuracy')
plt.legend()
plt.show()
