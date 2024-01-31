#!/usr/bin/env python3
import numpy as np
import os
import csv

max_num_files=10  # should be bigger than the number of runs per dataset 

def get_existing_files(prefix, max_num_files):
    files_out = []
    for ii in range(1,max_num_files):
        filename = prefix+str(ii)+'.txt'
        if os.path.isfile(filename):
            files_out.append(filename)
    return files_out

def get_stats(writer,subdirectory):
    files_ate = []
    files_resources = []
    rmse = []
    cpu = []
    ram = []

    # get existing files
    files_ate = get_existing_files('Ate_',max_num_files)    
    files_resources = get_existing_files('Resources_',max_num_files)
    
    for num, filename in enumerate(files_ate, 1):
        arr = np.genfromtxt(filename, dtype=None, usecols=(1), skip_header=1, max_rows= 1)
        rmse.append(arr.item(0))

    std_rmse = round(np.std(rmse),6)
    median_rmse = round(np.median(rmse),6)

    print(f'subdirectory: {subdirectory}')
    print(f'\t rmse: {rmse}')
    print(f'\t median_rmse: {median_rmse}, mean_rmse: {np.mean(rmse)}, std_rmse: {std_rmse}')
        
    for num, filename in enumerate(files_resources, 1):
        arr1 = np.genfromtxt(filename, dtype=None, usecols=(2), skip_header=1, delimiter=",")
        arr2 = np.genfromtxt(filename, dtype=None, usecols=(3), skip_header=1, delimiter=",")
        cpu.append(arr1.ravel())
        ram.append(arr2.ravel())
            
    cpu = np.concatenate(cpu, axis=0)
    ram = np.concatenate(ram, axis=0)

    cpu_mean = round(np.mean(cpu),2)
    cpu_std = round(np.std(cpu),2)
    ram_mean = round(np.mean(ram),2)
    ram_std = round(np.std(ram),2)

    writer.writerow([subdirectory, median_rmse, std_rmse, cpu_mean, cpu_std, ram_mean, ram_std,])


if __name__ == '__main__':
    root = os.getcwd()+'/results'
    print(f'root: {root}')
    os.chdir(root)
    csvfile= open("ResultsTable.csv", 'w')
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(["Dataset","RMSE","STD_RMSE","CPU","STD_CPU","RAM","STD_RAM"])
    
    dirlist = [ f for f in os.listdir(".") if os.path.isdir(f) and (not (f.endswith(".csv") or f.startswith('.'))) ]    
    for subdirectory in sorted(dirlist):
        os.chdir(os.path.join(root, subdirectory))
        get_stats(writer,subdirectory)
        os.chdir(root)

    csvfile.close()

