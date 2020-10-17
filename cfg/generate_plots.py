import numpy as np
import matplotlib.pyplot as plt
import csv
import bisect

def readInterpolatedValues(files, times, columns = []):
    
    first_file = True
    for filename in files:
        c0 = [] # Times
        try:
            with open(filename,'r') as csvfile:
                csv_reader = csv.reader(csvfile, delimiter=',')
                read_types = False
                if first_file:
                    column_names = next(csv_reader)
                    if not columns: # if no columns given, read all but time
                        columns = range(1,len(column_names))
                    column_names = [column_names[i] for i in columns]
                else:
                    next(csv_reader) # skip first line

                cs = [[] for _ in range(len(columns))] # colums to read
                for row in csv_reader:
                    c0.append(float(row[0]))
                    for i in range(len(cs)):
                        cs[i].append(float(row[columns[i]]))
        except IOError:
            print('File \'{}\' could not be opened; skipping file'.format(filename))
            continue 
    
        # interpolate data to whole seconds for easier averaging
        cs_n = [[] for _ in range(len(columns))] # colums adjusted to specified times
        ci = 0 # current index
        for t in times:
            if ci < len(times):
                ci = bisect.bisect_left(c0, t, ci)
            if ci >= len(times): # end of times reached; keep last value
                for i in range(len(cs_n)):
                    cs_n[i].append(cs[i][-1])
            elif ci == 0: # if t smaller than first value, keep first
                for i in range(len(cs_n)):
                    cs_n[i].append(cs[i][0])
            else:
                t1 = c0[ci - 1]
                t2 = c0[ci]
                for i in range(len(cs_n)):
                    val = cs[i][ci - 1] * (t - t1) / (t2 - t1) + cs[i][ci] * (t2 - t) / (t2 - t1)
                    cs_n[i].append(float(val))  
    
        if first_file:
            results = [[] for _ in range(len(columns))]

        for i in range(len(cs_n)):
            results[i].append(cs_n[i])

        first_file = False

    return results, column_names


auto_files = ['automatic/planner_results_0.csv', 'automatic/planner_results_1.csv', 'automatic/planner_results_2.csv', 'automatic/planner_results_3.csv', 'automatic/planner_results_4.csv', 'automatic/planner_results_5.csv', 'automatic/planner_results_6.csv', 'automatic/planner_results_7.csv', 'automatic/planner_results_8.csv', 'automatic/planner_results_9.csv']

contour_files = ['contours/planner_results_0.csv', 'contours/planner_results_1.csv', 'contours/planner_results_2.csv', 'contours/planner_results_3.csv', 'contours/planner_results_4.csv', 'contours/planner_results_5.csv', 'contours/planner_results_6.csv', 'contours/planner_results_7.csv', 'contours/planner_results_8.csv', 'contours/planner_results_9.csv']

times = range(181)

#colums_to_read = [1, 4, 5, 8]
#types_to_read = [int, float, float, int]

results_auto, plot_names = readInterpolatedValues(auto_files, times) #, colums_to_read, types_to_read)
results_contours, _ = readInterpolatedValues(contour_files, times) #, colums_to_read, types_to_read)

results_auto_avg = [np.average(results_auto[i], axis=0) for i in range(len(results_auto))]
results_contours_avg = [np.average(results_contours[i], axis=0) for i in range(len(results_contours))]

for i in range(len(plot_names)):
    plt.figure(i)

    for result in results_auto[i]:
        plt.plot(times, result, 'C1--', alpha=0.2)
    for result in results_contours[i]:
        plt.plot(times, result, 'C2--', alpha=0.2)

    plt.plot(times, results_auto_avg[i], 'C1', linewidth=3.0)
    plt.plot(times, results_contours_avg[i], 'C2', linewidth=3.0)

    plt.xlabel('Time (s)')
    plt.ylabel(plot_names[i])
    plt.title(plot_names[i])
    plt.legend()
    figname = plot_names[i].lower().replace(" ", "_") + ".png"
    plt.savefig(figname)
