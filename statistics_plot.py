from __future__ import division
import os
import IPython
import sys
import matplotlib.pyplot as plt
import numpy as np
plt.switch_backend('TKagg')


data_path = "/home/rui/Documents/research/iros_20_motoman_experiment/data/"

OSP_nCollision = 0.0
MCRG_nCollision = 0.0
MCRE_nCollision = 0.0
MSG_nCollision = 0.0
MSE_nCollision = 0.0
MCRMLC_nCollision = 0.0

OSP_success = 0.0
MCRG_success = 0.0
MCRE_success = 0.0
MSG_success = 0.0
MSE_success = 0.0
MCRMLC_success = 0.0

OSP_pathCost = 0.0
MCRG_pathCost = 0.0
MCRE_pathCost = 0.0
MSG_pathCost = 0.0
MSE_pathCost = 0.0
MCRMLC_pathCost = 0.0

OSP_time = 0.0
MCRG_time = 0.0
MCRE_time = 0.0
MSG_time = 0.0
MSE_time = 0.0
MCRMLC_time = 0.0

# image_list = os.listdir(data_path)
# nImage = len(image_list)

scene_idx = sys.argv[1]
if scene_idx == "1":
	image_list = ["806", "900", "904", "907", "910", "911", "912", "918", "921", "923", "929", "930", "931", "947", "948", "949", "950"]
	nImage = len(image_list)
if scene_idx == "2":
	image_list = ["902", "903", "905", "906", "909", "934", "935", "936", "942", "943", "944", "945", "946"]
	nImage = len(image_list)
if scene_idx == "3":
	image_list = ["901", "908", "932", "937", "938", "940", "941"] 
	nImage = len(image_list)


for img_index in image_list:
	stat_file = data_path + img_index + "/image_statistics.txt"
	f = open(stat_file)
	counter = 0
	for line in f:
		counter += 1
		line = line.split()
		if counter == 1:
			OSP_nCollision += float(line[0])
			OSP_success += float(line[1])
			OSP_pathCost += float(line[2])
			OSP_time += float(line[3])
		if counter == 2:
			MCRG_nCollision += float(line[0])
			MCRG_success += float(line[1])
			MCRG_pathCost += float(line[2])
			MCRG_time += float(line[3])
		if counter == 3:
			MCRE_nCollision += float(line[0])
			MCRE_success += float(line[1])
			MCRE_pathCost += float(line[2])
			MCRE_time += float(line[3])
		if counter == 4:
			MSG_nCollision += float(line[0])
			MSG_success += float(line[1])
			MSG_pathCost += float(line[2])
			MSG_time += float(line[3])
		if counter == 5:
			MSE_nCollision += float(line[0])
			MSE_success += float(line[1])
			MSE_pathCost += float(line[2])
			MSE_time += float(line[3])
		if counter == 6:
			MCRMLC_nCollision += float(line[0])
			MCRMLC_success += float(line[1])
			MCRMLC_pathCost += float(line[2])
			MCRMLC_time += float(line[3])


OSP_nCollision /= nImage
OSP_success /= nImage
OSP_pathCost /= nImage
OSP_time /= nImage
MCRG_nCollision /= nImage
MCRG_success /= nImage
MCRG_pathCost /= nImage
MCRG_time /= nImage
MCRE_nCollision /= nImage
MCRE_success /= nImage
MCRE_pathCost /= nImage
MCRE_time /= nImage
MSG_nCollision /= nImage
MSG_success /= nImage
MSG_pathCost /= nImage
MSG_time /= nImage
MSE_nCollision /= nImage
MSE_success /= nImage
MSE_pathCost /= nImage
MSE_time /= nImage
MCRMLC_nCollision /= nImage
MCRMLC_success /= nImage
MCRMLC_pathCost /= nImage
MCRMLC_time /= nImage

print("OSP average collision: ", OSP_nCollision)
print("MCRG average collision: ", MCRG_nCollision)
print("MCRE average collision: ", MCRE_nCollision)
print("MSG average collision: ", MSG_nCollision)
print("MSE average collision: ", MSE_nCollision)
print("MCRMLC average collision: ", MCRMLC_nCollision)
print("\n")

print("OSP average success: ", OSP_success)
print("MCRG average success: ", MCRG_success)
print("MCRE average success: ", MCRE_success)
print("MSG average success: ", MSG_success)
print("MSE average success: ", MSE_success)
print("MCRMLC average success: ", MCRMLC_success)
print("\n")

print("OSP average pathCost: ", OSP_pathCost)
print("MCRG average pathCost: ", MCRG_pathCost)
print("MCRE average pathCost: ", MCRE_pathCost)
print("MSG average pathCost: ", MSG_pathCost)
print("MSE average pathCost: ", MSE_pathCost)
print("MCRMLC average pathCost: ", MCRMLC_pathCost)
print("\n")

print("OSP average time: ", OSP_time)
print("MCRG average time: ", MCRG_time)
print("MCRE average time: ", MCRE_time)
print("MSG average time: ", MSG_time)
print("MSE average time: ", MSE_time)
print("MCRMLC average time: ", MCRMLC_time)
print("\n")


### Now let's draw the bar graph 
FIGSIZE_1 = (3.45, 1.85)
FIGSIZE = (2, 1.8)
FONTSIZE = 7
LABELSIZE = 6
# MARKERSIZE = 4
LINEWIDTH = 1
plt.rcParams["legend.labelspacing"] = 0.2
plt.rcParams["legend.handlelength"] = 1.75
plt.rcParams["legend.handletextpad"] = 0.5
plt.rcParams["legend.columnspacing"] = 0.75

plt.rcParams.update({'figure.autolayout': True})

plt.rcParams['ps.useafm'] = True
plt.rcParams['pdf.use14corefonts'] = True
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
plt.rcParams['font.serif'] = "Times"


x = np.array([i * 100 + 100 for i in range(0, 1)])

fig = plt.figure(1, figsize = FIGSIZE_1)
ax = fig.add_subplot(111)

bar_width = 6
space = 10

ax.bar(x - space * 2.5, OSP_success*100, bar_width, label = "Optimistic Shortest Path", edgecolor = "black", color = "red")
ax.bar(x - space * 1.5, MCRG_success*100, bar_width, label = "MCR-Greedy", edgecolor = "black", color = "green")
ax.bar(x - space * 0.5, MCRE_success*100, bar_width, label = "MCR-Exact", edgecolor = "black", color = "cyan")
ax.bar(x + space * 0.5, MCRMLC_success*100, bar_width, label = "MCR-Most Likely Candidate", edgecolor = "black", color = "orange")
ax.bar(x + space * 1.5, MSG_success*100, bar_width, label = "MaxSuccess-Greedy", edgecolor = "black", color = "pink")
ax.bar(x + space * 2.5, MSE_success*100, bar_width, label = "MaxSuccess-Exact", edgecolor = "black", color = "purple")

ax.set_ylabel("Success Rate(\%)", fontsize = FONTSIZE)
ax.tick_params(labelsize = FONTSIZE)
ax.set_ylim(0, 120)
ax.legend(fontsize = LABELSIZE, ncol = 2, loc="best")
ax.yaxis.grid(True, alpha = 0.99)
ax.set_axisbelow(True)
ax.xaxis.set_label_coords(0.5, -0.15)
ax.set_xticks([])
# if scene_idx == "1":
# 	ax.set_xlabel("Scenario 1: target in objects clutter", fontsize = FONTSIZE)
# if scene_idx == "2":
# 	ax.set_xlabel("Scenario 2: target in narrow passage", fontsize = FONTSIZE)
# if scene_idx == "3":
# 	ax.set_xlabel("Scenario 3: target in front of objects arch", fontsize = FONTSIZE)
fig.savefig("/home/rui/Documents/research/iros_20_motoman_experiment/figures/success_rate_s"+scene_idx+".png", bbox_inches="tight", pad_inches=0.05)

fig = plt.figure(2, figsize = FIGSIZE_1)
ax = fig.add_subplot(111)

bar_width = 6
space = 10

ax.bar(x - space * 2.5, OSP_nCollision, bar_width, label = "Optimistic Shortest Path", edgecolor = "black", color = "red")
ax.bar(x - space * 1.5, MCRG_nCollision, bar_width, label = "MCR-Greedy", edgecolor = "black", color = "green")
ax.bar(x - space * 0.5, MCRE_nCollision, bar_width, label = "MCR-Exact", edgecolor = "black", color = "cyan")
ax.bar(x + space * 0.5, MCRMLC_nCollision, bar_width, label = "MCR-Most Likely Candidate", edgecolor = "black", color = "orange")
ax.bar(x + space * 1.5, MSG_nCollision, bar_width, label = "MaxSuccess-Greedy", edgecolor = "black", color = "pink")
ax.bar(x + space * 2.5, MSE_nCollision, bar_width, label = "MaxSuccess-Exact", edgecolor = "black", color = "purple")

ax.set_ylabel("number of objects collided", fontsize = FONTSIZE)
ax.tick_params(labelsize = FONTSIZE)
ax.set_ylim(0, 1.2)
ax.legend(fontsize = LABELSIZE, ncol = 2, loc="best")
ax.yaxis.grid(True, alpha = 0.99)
ax.set_axisbelow(True)
ax.xaxis.set_label_coords(0.5, -0.15)
ax.set_xticks([])
# if scene_idx == "1":
# 	ax.set_xlabel("Scenario 1: target in objects clutter", fontsize = FONTSIZE)
# if scene_idx == "2":
# 	ax.set_xlabel("Scenario 2: target in narrow passage", fontsize = FONTSIZE)
# if scene_idx == "3":
# 	ax.set_xlabel("Scenario 3: target in front of objects arch", fontsize = FONTSIZE)
fig.savefig("/home/rui/Documents/research/iros_20_motoman_experiment/figures/ncollisions_s"+scene_idx+".png", bbox_inches="tight", pad_inches=0.05)

plt.show()
