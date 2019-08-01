#!/usr/bin/python
import os, sys
import random
import shutil
import subprocess as sp
import time

VERBOSE=False
RUN_BASELINE=False

# Number of simulations per Demand/Parameter configuration
Nsim = 1
# Number of lanes (regarding the LOS) in the current scenario
Nlanes = 2

# Number of restarts if a run fails
MAXTRIES = 0

# List of genral additional files to be loaded for all runs
generalAddFiles = ["../../../view.add.xml"]

# Demand levels (levelID->veh/(h*l))
demand_motorway = {"los_A":735, "los_B":1155, "los_C":1617}
demand_motorway = {"los_C":1617}

# levels to be used for simulations
demand_levels = demand_motorway    
demand_ID_map = {"los_A":0, "los_B":1, "los_C":2}
demand_ID_map = {"los_C":2}

# Vehicle mix (mix_ID->class->share [in %])
veh_mixes = {"mix_0":{"LV":0.7, "CVToC":0.15, "CAVToC":0.15},
             "mix_1":{"LV":0.5, "CVToC":0.25, "CAVToC":0.25},
             "mix_2":{"LV":0.2, "CVToC":0.4, "CAVToC":0.4}}
veh_mixes = {"mix_2":{"LV":0.2, "CVToC":0.4, "CAVToC":0.4}}

# Parameter assumptions regarding (E)fficiency/(S)afety and (O)ptimism/(P)essimism
# param_schemes = ["PS", "OS", "PE", "OE", "MSE"]
param_schemes = ["MSE"]

# template file for the routes
routefile_template = "routes_template.rou.xml"

# config file
config_file = "sumo.cfg"

# detectors file
detectors_template = "detectors_template.add.xml"

# runner script for single runs
runner_file = "runner.py"

def fillTemplate(template_file, out_file, content_dict):
    with open(template_file) as f:
        template = f.read()
        filled_template = template.format(**content_dict)
    with open(out_file, "w") as f:
        f.write(filled_template)
    
if __name__ == "__main__":
    if "--baseline" in sys.argv:
        print("\n### Running baseline! ###\n")
        time.sleep(1.0)
        RUN_BASELINE=True
    wdir = os.path.dirname(os.path.realpath(__file__))
    print("Working dir = %s"%wdir)
    # nr of runs started from this script
    runCount = 0
    # remove vTypeDistribution files and create them again
    if "--generateVTypes" in sys.argv: sp.call(["python", "distribution.py"])
    
    # remove existing subdir structure
    for d in demand_levels:
        try:
            shutil.rmtree(os.path.join(wdir, d))
            print("removing dir %s"%d)
        except:
            pass

    # Create subdirectory structure: params/mix/demand/; fail if it exists
    # ... and start simulations
    start_time = time.time()
    # Total number of run-groups (Nrun runs per parameter combination) to be conducted (for ETA calculation)
    NRunsTotal = len(demand_levels)*len(veh_mixes)*len(param_schemes)
    NRunsTotalRemaining = NRunsTotal
    print("Total parameter-combinations to be scanned: %s"%NRunsTotal)
    for d in demand_levels:
        print("Demand: %s"%d)
        ddir = os.path.join(wdir, d)
        os.mkdir(ddir)
        for m in veh_mixes:
            print("Mix: %s"%m)
            mdir = os.path.join(ddir, m)
            os.mkdir(mdir)
            for p in param_schemes:
                print("Scheme: %s"%p)
                pdir = os.path.join(mdir, p)
                os.mkdir(pdir)
                os.mkdir(os.path.join(pdir, "results"))
                
                # fill and copy sumo config
                addFileList = ["vTypes"+s+"_"+p+".add.xml" for s in veh_mixes[m].keys()]
                #~ addFileDict = {"addFiles": ", ".join(addFileList)}
                #~ print("addFileDict: %s"%addFileDict)
                config_fn = os.path.join(pdir, config_file)
                #~ fillTemplate(os.path.join(wdir, config_file), config_fn, addFileDict)
                fillTemplate(os.path.join(wdir, config_file), config_fn, {})
                
                # copy vType files
                for fn in addFileList:
                    shutil.copyfile(os.path.join(wdir, fn), os.path.join(pdir, fn))
        
                # copy runner script
                runner_fn = os.path.join(pdir, runner_file)
                shutil.copyfile(os.path.join(wdir, runner_file), os.path.join(pdir, runner_fn))
                
                # run simulations
                working_set = {}
                for i in xrange(Nsim):
                    os.chdir(pdir)
                    
                    output_suffix = "_trafficMix_%s_trafficDemand_%s_driverBehaviour_%s_seed_%s.xml"%(m[-1], demand_ID_map[d], p, i)
                    
                    # Fill and copy run specific routes file
                    route_dict = dict([(vType+"prob", Nlanes*percentage*demand_levels[d]/3600.) for (vType, percentage) in veh_mixes[m].iteritems()])
                    route_dict.update(dict([(vType+"type", "veh"+vType+p) for vType in veh_mixes[m]]))
                    ssmOutFile = "outputSSM" + output_suffix
                    route_dict.update({"ssmOutFile" : os.path.join(pdir,  ssmOutFile)})
                    routes_fn = "routes" + output_suffix
                    routes_fn = os.path.join(pdir, routes_fn)
                    fillTemplate(os.path.join(wdir, routefile_template), routes_fn, route_dict)
                    if VERBOSE:
                        print("wrote file '%s'\nrouteDict=%s"%(os.path.join(pdir, routes_fn), route_dict))
                    
                    # copy detectors script
                    detectors_fn = "detectors"+output_suffix
                    detectors_fn = detectors_fn.replace(".xml" , ".add.xml")
                    detectors_fn = os.path.join(pdir, detectors_fn)
                    detectors_dict={"trajectoriesSuffix":output_suffix,
                                    "detectorsSuffix":output_suffix }
                    fillTemplate(os.path.join(wdir, detectors_template), detectors_fn, detectors_dict)

                    # Fill and copy run specific aggregated output additional file
                    output_addFileTemplate = "additionalOutput_template.add.xml"
                    output_addFile = "additionalsOutput" + output_suffix
                    output_detectors = detectors_fn
                    emissionOutputFile = os.path.join(pdir, "outputEmission" + output_suffix)
                    meandataOutputFile = os.path.join(pdir, "outputMeandata" + output_suffix)
                    fillTemplate(os.path.join(wdir, output_addFileTemplate), os.path.join(pdir, output_addFile), {"emissionOutfile":emissionOutputFile, "meandataOutfile":meandataOutputFile})
                    allAddFiles = ", ".join(addFileList+[output_addFile]+generalAddFiles+[output_detectors])
                    
                    ## Run with fixed seeds (for reproducibility)
                    seed = str(1042 + i)
                    if "--gui" in sys.argv:
                        argv = ["python", runner_fn, "-v", "--gui","-c", config_fn, "--additionals", allAddFiles, "--routes", routes_fn, "--seed", seed, "--suffix", output_suffix]
                    else:
                        argv = ["python", runner_fn, "-c", config_fn, "--additionals", allAddFiles, "--routes", routes_fn, "--seed", seed, "--suffix", output_suffix]
                    
                    if RUN_BASELINE:
                        argv.append("--baseline")
                        
                    if VERBOSE:
                        print("Calling: %s"%argv)
                    
                    
                    ## Call one at a time writing to stdout (for testing)
                    #~ Ntries = 0
                    #~ while(sp.call(argv) != 0 and Ntries < MAXTRIES):
                        #~ Ntries += 1
                    
                    ## Call in separate processes, writing to output file
                    out_fn=os.path.join(pdir, "output_"+str(i)+".txt")
                    if VERBOSE:
                        print("Writing output to file '%s'"%out_fn)
                    outfile = open(out_fn, "w")
                    working_set[i] = (sp.Popen(argv, stdout=outfile, stderr=outfile), outfile, argv)
                    runCount += 1
                    
                
                print("\nWaiting for all simulations for %s->%s->%s to complete...\n"%(d, m, p))
                
                # wait for all run to end, restart failed calls 
                # (FIXME: reason for failing is unclear and fails are not reliably reproducible)
                # -> Might be resolved
                Ntries = 0
                while any(working_set):
                    i, v = working_set.popitem()
                    waitProcess, outfile, argv = v
                    returnCode = waitProcess.poll()
                    if returnCode is None:
                        # process not yet finished, push it back
                        working_set[i]=v
                        #time.sleep(0.1)
                        continue
                        
                    returnCode = waitProcess.wait()
                    outfile.close()
                    if returnCode != 0 and Ntries < MAXTRIES:
                        # restart process
                        print ("Run %s failed with code %s. (outfile='%s') Restarting..."%(i, returnCode, outfile))
                        Ntries += 1
                        out_fn=outfile.name
                        outfile = open(out_fn, "w")
                        working_set[i] = (sp.Popen(argv, stdout=outfile, stderr=outfile), outfile, argv)
                    else:
                        print ("Run %s completed..."%i)
                
                
                if not "--no-gzip" in sys.argv: 
                    time.sleep(3.0) # don't gzip config files before read 
                    # gzip all output for this run
                    callList=["gzip", "-r9"] + [os.path.join(pdir, fn) for fn in os.listdir(pdir)]
                    print("\ngzipping all files in "+pdir+"...\n")
                    sp.call(callList)
                    
                # Report ETA
                NRunsTotalRemaining -= 1
                elapsed = time.time()-start_time
                estimatedPerRun =  elapsed/(NRunsTotal-NRunsTotalRemaining)
                estimatedRemaining = estimatedPerRun*NRunsTotalRemaining
                ETA = time.gmtime(time.time() + estimatedRemaining)
                print ("\n# Estimated time remaining: %s h. (ETA: %s)\n"%(estimatedRemaining/3600., str(ETA.tm_mon)+"-"+str(ETA.tm_mday)+", "+ str(ETA.tm_hour+1)+":"+str(ETA.tm_min)+":"+str(ETA.tm_sec)))
                
                
    # report elapsed time
    end_time = time.time()
    elapsed = end_time-start_time
    print ("\nDone.\n\nElapsed time: %s (for %s runs)\n"%(elapsed, runCount))
    #~ # compress output
    #~ try: shutil.rmtree(os.path.join(wdir, "output.zip"))
    #~ except: pass
    #~ zip_command = ["zip", os.path.join(wdir,"output.zip"), os.path.join(wdir,"los_*"), "-r"]
    #~ print("Calling "+" ".join(zip_command))
    #~ sp.call(["zip", os.path.join(wdir,"output.zip"), os.path.join(wdir,"los_*"), "-r"])
                
                
