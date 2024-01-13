#**
# ******************************************************************************
# * @file           : FOCAlgoLogParser.py
# * @brief          : Python parser to read out data from serial log to a CSV
# * @author         : Sreedhar Mahadevan
# * @company        : Agnikul Cosmos Private Limited
# ******************************************************************************
# *
# * @attention
# *
# * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
# * All rights reserved.</center></h2>
# *
# * This software component is licensed by ST under BSD 3-Clause license,
# * the "License"; You may not use this file except in compliance with the
# * License. You may obtain a copy of the License at:
# *                        opensource.org/licenses/BSD-3-Clause
# *
# ******************************************************************************
# */

import csv

# Modify the name of the log file everytime before running script
infile = r"20210316_focalgo.log"

print
print
print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
print("******** FOC ALGORITHM ON NXP-FRDMKV31F LOG PARSER *********")
print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
print
print

print
print("Parsing '" + infile + "' log file. Please change the file name if this is not the intended file...")
print
print

Heading = ['Iterations', 'Ud-NXP', 'Uq-NXP']
# Variables to store the extracted values from the log file

iterations = []
Ud_NXP = []
Uq_NXP = []
line_count = 0

with open(infile) as f:
    f = f.readlines()

for line in f:
#   Iterations extracted
    if 'i:' in line:
        iterations.append(line.split()[1])
        
#   Ud and Uq values extracted
    if 'Ud:' in line:
        line_count+=1
        Ud_NXP.append(line.split()[1])
        Uq_NXP.append(line.split()[3])
        
print
print("Number of log entries to be written to CSV: " + str(line_count))
print

print ("Ud-NXP: ")
print Ud_NXP
print

print ("Uq-NXP: ")
print Uq_NXP
print

csvname = infile + ".csv"

with open(csvname, 'wb') as csvfile:
    writer = csv.writer(csvfile, delimiter=',')
    writer.writerow(Heading)
       
    for elements_index in range(0, line_count):
        Elements = [] 
        Elements.append(iterations[elements_index])
        Elements.append(Ud_NXP[elements_index])
        Elements.append(Uq_NXP[elements_index])
        writer.writerow(Elements)
        
print
print
print
print("All useful data are summarized in '" + csvname + "' file in the same folder.")
print
