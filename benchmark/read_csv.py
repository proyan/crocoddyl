"""
Reads csv benchmarks provided as input. Creates a DataFrame for each benchmark, and prints the table for easy reading.
The benchmarks are stored in the object benches

Depends:           pandas module (pip install pandas)

How to Run:        python -i read_csv.py /tmp/bench1.bench /tmp/bench2.bench ...
                   e.g.: ipython -i read_csv.py /tmp/Anymal_19DoF.bench /tmp/HyQ_19DoF.bench
                   e.g.: ipython -i read_csv.py /tmp/*.bench


Output:            benches
                   benches.bench1 (DataFrame), benches.bench2 (DataFrame)...

DataFrame API:     value = benches.bench1.loc[fn_name, nthreads, with(out)_cg][bench_parameter]
                   example:
                   mean_calc_time_withcg_with3threads = benches.bench1.loc["calc",3,True]["mean"]
                   max_calc_time_withoutcg_with2threads = benches.bench1.loc["calc",2,False]["max"]

"""

from __future__ import print_function
import pandas as pd
import sys
from os.path import exists, splitext, basename
import numpy as np
pd.options.display.width = 0


class Benchmarks():
    pass


def parseCsvFile(filename, sep=',', delimiter=None):
    """Inputs a file and ouputs a matrix.
    input: (string) filename
    output: (numpy array) seq
    """
    col_types = dict(fn_name=str,
                     nthreads=int,
                     with_cg=bool,
                     mean=float,
                     stddev=float,
                     max=float,
                     min=float,
                     mean_per_nodes=float,
                     stddev_per_nodes=float)

    seq = pd.read_csv(filename,
                      header=0,
                      sep=sep,
                      delim_whitespace=False,
                      quoting=2,
                      index_col=[0, 1, 2],
                      dtype=col_types)

    return seq


benches = Benchmarks()

filenames = sys.argv[1:]
for filename in filenames:
    if not exists(filename):
        continue
    benchname = splitext(basename(filename))[0]
    seq = parseCsvFile(filename)
    setattr(benches, benchname, seq)
    print(benchname)
    print(seq)
    print("********************")


import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111)

previous_benches = ['Talos_arm_4DoF',
                    'Kinova_arm_9DoF',
                    'Solo_15DoF',
                    'Anymal_19DoF',
                    'HyQ_19DoF',
                    'Talos_29DoF',
                    'iCub_36DoF']

new_benches = ['Talos_arm_4DoF_constrained',
               'Kinova_arm_9DoF_constrained',
               'Solo_15DoF_constrained',
               'Anymal_19DoF_constrained',
               'HyQ_19DoF_constrained',
               'Talos_29DoF_constrained',
               'iCub_36DoF_constrained']

Nr = 7
ind = np.arange(Nr)                # the x locations for the groups
width = 0.35                      # the width of the bars

calcMeansPrev = [benches.__dict__[bench].loc["calc",1,False]["mean"] for bench in previous_benches]
calcStdPrev = [benches.__dict__[bench].loc["calc",1,False]["stddev"] for bench in previous_benches]
calcMeansNew = [benches.__dict__[bench].loc["calc",1,False]["mean"] for bench in new_benches]
calcStdNew = [benches.__dict__[bench].loc["calc",1,False]["stddev"] for bench in new_benches]


rects1 = ax.bar(ind, calcMeansPrev, width,
                color='blue',
                yerr=calcStdPrev,
                error_kw=dict(elinewidth=2,ecolor='red'))

rects2 = ax.bar(ind+width, calcMeansNew, width,
                color='red',
                yerr=calcStdNew,
                error_kw=dict(elinewidth=2,ecolor='black'))

# axes and labels
ax.set_xlim(-width,len(ind)+width)
#ax.set_ylim(max(calcMeansPrev))
ax.set_ylabel('Time (in us)')
ax.set_title('Performance of Contact Dynamics for 100 calcs with 1 thread without codegen')
xTickMarks = ['Talos 4DoF', 'Kinova 9DoF', 'Solo 15DoF', 'Anymal 19DoF', 'HyQ 19DoF', 'Talos 29DoF', 'iCub 36DoF']
ax.set_xticks(ind+width)
xtickNames = ax.set_xticklabels(xTickMarks)
plt.setp(xtickNames, rotation=45, fontsize=10)

ax.legend( (rects1[0], rects2[0]), ('Previous', 'New') )

fig.savefig('calc.png')#, dpi=fig.dpi)
plt.show()

#plt.clf()
#fig.clear()

##########################################################################################################

fn_name = "calcDiff"
nthreads = 1
with_codegen = False

fig = plt.figure()
ax = fig.add_subplot(111)
calcDiffMeansPrev = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["mean"] for bench in previous_benches]
calcDiffStdPrev = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["stddev"] for bench in previous_benches]
calcDiffMeansNew = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["mean"] for bench in new_benches]
calcDiffStdNew = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["stddev"] for bench in new_benches]


rects1 = ax.bar(ind, calcDiffMeansPrev, width,
                color='blue',
                yerr=calcDiffStdPrev,
                error_kw=dict(elinewidth=2,ecolor='red'))

rects2 = ax.bar(ind+width, calcDiffMeansNew, width,
                color='red',
                yerr=calcDiffStdNew,
                error_kw=dict(elinewidth=2,ecolor='black'))
# axes and labels
ax.set_xlim(-width,len(ind)+width)
#ax.set_ylim(max(calcMeansPrev))
ax.set_ylabel('Time (in us)')
ax.set_title('Performance of Contact Dynamics for 100 calcDiffs with 1 thread without codegen')
xTickMarks = ['Talos 4DoF', 'Kinova 9DoF', 'Solo 15DoF', 'Anymal 19DoF', 'HyQ 19DoF', 'Talos 29DoF', 'iCub 36DoF']
ax.set_xticks(ind+width)
xtickNames = ax.set_xticklabels(xTickMarks)
plt.setp(xtickNames, rotation=45, fontsize=10)

ax.legend( (rects1[0], rects2[0]), ('Previous', 'New') )

fig.savefig('calcDiff.png')#, dpi=fig.dpi)
plt.show()

####################################################################################################################
fn_name = "calcDiff"
nthreads = 1
with_codegen = True

fig = plt.figure()
ax = fig.add_subplot(111)
calcDiffMeansPrev = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["mean"] for bench in previous_benches]
calcDiffStdPrev = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["stddev"] for bench in previous_benches]
calcDiffMeansNew = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["mean"] for bench in new_benches]
calcDiffStdNew = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["stddev"] for bench in new_benches]


rects1 = ax.bar(ind, calcDiffMeansPrev, width,
                color='blue',
                yerr=calcDiffStdPrev,
                error_kw=dict(elinewidth=2,ecolor='red'))

rects2 = ax.bar(ind+width, calcDiffMeansNew, width,
                color='red',
                yerr=calcDiffStdNew,
                error_kw=dict(elinewidth=2,ecolor='black'))
# axes and labels
ax.set_xlim(-width,len(ind)+width)
#ax.set_ylim(max(calcMeansPrev))
ax.set_ylabel('Time (in us)')
ax.set_title("Performance of Contact Dynamics for 100 calcDiffs with"+str(nthreads)+" threads with codegen")
xTickMarks = ['Talos 4DoF', 'Kinova 9DoF', 'Solo 15DoF', 'Anymal 19DoF', 'HyQ 19DoF', 'Talos 29DoF', 'iCub 36DoF']
ax.set_xticks(ind+width)
xtickNames = ax.set_xticklabels(xTickMarks)
plt.setp(xtickNames, rotation=45, fontsize=10)
ax.legend( (rects1[0], rects2[0]), ('Previous', 'New') )

fig.savefig('calcDiffTrue.png')#, dpi=fig.dpi)
plt.show()


################################################################################################################

fn_name = "calc"
nthreads = 1
with_codegen = True

fig = plt.figure()
ax = fig.add_subplot(111)
calcDiffMeansPrev = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["mean"] for bench in previous_benches]
calcDiffStdPrev = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["stddev"] for bench in previous_benches]
calcDiffMeansNew = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["mean"] for bench in new_benches]
calcDiffStdNew = [benches.__dict__[bench].loc[fn_name,nthreads,with_codegen]["stddev"] for bench in new_benches]


rects1 = ax.bar(ind, calcDiffMeansPrev, width,
                color='blue',
                yerr=calcDiffStdPrev,
                error_kw=dict(elinewidth=2,ecolor='red'))

rects2 = ax.bar(ind+width, calcDiffMeansNew, width,
                color='red',
                yerr=calcDiffStdNew,
                error_kw=dict(elinewidth=2,ecolor='black'))
# axes and labels
ax.set_xlim(-width,len(ind)+width)
#ax.set_ylim(max(calcMeansPrev))
ax.set_ylabel('Time (in us)')
ax.set_title("Performance of Contact Dynamics for 100 calcs with "+str(nthreads)+" threads with codegen")
xTickMarks = ['Talos 4DoF', 'Kinova 9DoF', 'Solo 15DoF', 'Anymal 19DoF', 'HyQ 19DoF', 'Talos 29DoF', 'iCub 36DoF']
ax.set_xticks(ind+width)
xtickNames = ax.set_xticklabels(xTickMarks)
plt.setp(xtickNames, rotation=45, fontsize=10)
ax.legend( (rects1[0], rects2[0]), ('Previous', 'New') )

fig.savefig('calcTrue.png')#, dpi=fig.dpi)
plt.show()


################################################################################################################

fn_name = "calc"
#nthreads = 1
with_codegen = True
benchname = benches.iCub_36DoF

Nt=16

fig = plt.figure()
ax = fig.add_subplot(111)
calcMeans = [benchname.loc["calc",nt,not with_codegen]["mean"] for nt in range(1,Nt+1)]
calcMeansStd = [benchname.loc["calc",nt,not with_codegen]["stddev"] for nt in range(1,Nt+1)]
calcMeanscg = [benchname.loc["calc",nt,with_codegen]["mean"] for nt in range(1,Nt+1)]
calcMeansStdcg = [benchname.loc["calc",nt,with_codegen]["stddev"] for nt in range(1,Nt+1)]


calcDiffMeans = [benchname.loc["calcDiff",nt,not with_codegen]["mean"] for nt in range(1,Nt+1)]
calcDiffMeansStd = [benchname.loc["calcDiff",nt,not with_codegen]["stddev"] for nt in range(1,Nt+1)]
calcDiffMeanscg = [benchname.loc["calcDiff",nt,with_codegen]["mean"] for nt in range(1,Nt+1)]
calcDiffMeansStdcg = [benchname.loc["calcDiff",nt,with_codegen]["stddev"] for nt in range(1,Nt+1)]

ind = np.arange(Nt)
width = 0.8

rects1 = ax.bar(ind, calcDiffMeans, width,
                color='blue',
                yerr=calcDiffMeansStd,
                error_kw=dict(elinewidth=2,ecolor='red', alpha=0.5), alpha=0.5)

rects1cg = ax.bar(ind, calcDiffMeanscg, width,
                color='blue',
                yerr=calcDiffMeansStdcg,
                error_kw=dict(elinewidth=2,ecolor='red', alpha=0.5))

rects2 = ax.bar(ind+Nt, calcMeans, width,
                color='red',
                yerr=calcMeansStd,
                error_kw=dict(elinewidth=2,ecolor='black', alpha=0.5), alpha=0.5)

rects2cg = ax.bar(ind+Nt, calcMeanscg, width,
                color='red',
                yerr=calcMeansStd,
                error_kw=dict(elinewidth=2,ecolor='black', alpha=0.5))

ax.set_xlim(-width,Nt+width)
#ax.set_ylim(max(calcMeansPrev))
ax.set_ylabel('Time (in us)')
ax.set_title("Performance of Multithreading and Code-generation for Talos calc and calcdiff functions")
xTickMarks = [str(i) for i in range(1,Nt+1)] * 2
ax.set_xticks(np.arange(2*Nt))
xtickNames = ax.set_xticklabels(xTickMarks)
ax.set_xlabel('Number of threads')
plt.setp(xtickNames, fontsize=10)
ax.legend( (rects1[0], rects1cg[0], rects2[0], rects2cg[0]), ('CalcDiff', 'CalcDiffCg', 'Calc', 'CalcCg') )

fig.savefig('calcTrue.png')#, dpi=fig.dpi)
plt.show()

################################################################################################################

#nthreads = 1
benchname = benches.Talos_29DoF

Nt=16

fig = plt.figure()
ax = fig.add_subplot(111)
calcMeans = [benchname.loc["calc",nt,not with_codegen]["mean"] for nt in range(1,Nt+1)]
calcMeansStd = [benchname.loc["calc",nt,not with_codegen]["stddev"] for nt in range(1,Nt+1)]
calcMeanscg = [benchname.loc["calc",nt,with_codegen]["mean"] for nt in range(1,Nt+1)]
calcMeansStdcg = [benchname.loc["calc",nt,with_codegen]["stddev"] for nt in range(1,Nt+1)]

calcDiffMeans = [benchname.loc["calcDiff",nt,not with_codegen]["mean"] for nt in range(1,Nt+1)]
calcDiffMeansStd = [benchname.loc["calcDiff",nt,not with_codegen]["stddev"] for nt in range(1,Nt+1)]
calcDiffMeanscg = [benchname.loc["calcDiff",nt,with_codegen]["mean"] for nt in range(1,Nt+1)]
calcDiffMeansStdcg = [benchname.loc["calcDiff",nt,with_codegen]["stddev"] for nt in range(1,Nt+1)]

backwardPassMean = benchname.loc["backwardPass",1,with_codegen]["mean"]
backwardPassStdDev = benchname.loc["backwardPass",1,with_codegen]["stddev"]
forwardPassMean = benchname.loc["forwardPass",1,with_codegen]["mean"]
forwardPassStdDev = benchname.loc["forwardPass",1,with_codegen]["stddev"]


backwardPassMin = benchname.loc["backwardPass",1,with_codegen]["min"]
backwardPassMax = benchname.loc["backwardPass",1,with_codegen]["max"]
forwardPassMin = benchname.loc["forwardPass",1,with_codegen]["min"]
forwardPassMax = benchname.loc["forwardPass",1,with_codegen]["max"]

calcDiffcgMin = [benchname.loc["calcDiff",nt,with_codegen]["min"] for nt in range(1,Nt+1)]
calcDiffcgMax = [benchname.loc["calcDiff",nt,with_codegen]["max"] for nt in range(1,Nt+1)]
calccgMin = [benchname.loc["calc",nt,with_codegen]["min"] for nt in range(1,Nt+1)]
calccgMax = [benchname.loc["calc",nt,with_codegen]["max"] for nt in range(1,Nt+1)]



fig, ax = plt.subplots(figsize=(16, 10))

stacks = pd.DataFrame([[backwardPassMean, calcDiffMeanscg[i], forwardPassMean, calcMeanscg[i]] for i in xrange(Nt)])
stacks_stddev = pd.DataFrame([[backwardPassStdDev, calcDiffMeansStdcg[i], forwardPassStdDev, calcMeansStdcg[i]] for i in xrange(Nt)])
stacks_stddev = pd.DataFrame([[backwardPassStdDev, calcDiffMeansStdcg[i], forwardPassStdDev, calcMeansStdcg[i]] for i in xrange(Nt)])

stacks_min = pd.DataFrame([[backwardPassMin, calcDiffcgMin[i], forwardPassMin, calccgMin[i]] for i in xrange(Nt)])
stacks_max = pd.DataFrame([[backwardPassMax, calcDiffcgMax[i], forwardPassMax, calccgMax[i]] for i in xrange(Nt)])

stacks = stacks.rename(columns={0:'backwardPass', 1:'calcDiff', 2:'forwardPass', 3:'calc'})
stacks_stddev = stacks_stddev.rename(columns={0:'backwardPass', 1:'calcDiff', 2:'forwardPass', 3:'calc'})



import scipy.stats as stats

normal = [stats.norm(sum(stacks.loc[i], sum(stacks_stddev.loc[i]))) for i in xrange(Nt)]




stacks.plot.bar(stacked=True, ax=ax)
plt.suptitle("Performance of Code-generated Talos 28DoF robot for one ddp iteration")

for i,rect in enumerate(ax.patches[-Nt:]):
    height = sum(stacks.loc[i])
    print(height)
    freq = 1000000./height
    ax.annotate(str(int(freq))+ "Hz", xy=(rect.get_x()+rect.get_width()/2, height),
                xytext=(0, 5), textcoords='offset points', ha='center', va='bottom')
ax.set_xlabel('Number of threads')
ax.set_ylabel('Time (in us)')
xTickMarks = [str(i) for i in range(1,Nt+1)]
ax.set_xticks(np.arange(Nt))
xtickNames = ax.set_xticklabels(xTickMarks)

plt.show()
