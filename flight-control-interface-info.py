# NOTES FOR RECON-FLIGHT CONTROL INTERFACE
# James, 05/05/2018

# For Oliver Gent
# Thanks for your help, let me know if you have any questions!

# Interface can be defined however you like - we just need functions that implement these commands:
#
# get current location							get current heading/bearing
# set waypoint									set heading/bearing
# callback/notification on waypoint reached		notification on take-off sequence complete
# start landing sequence						send x via telemetry
#
# (there are intricies to do with timing/implementation etc. which should be discussed but we can do that later)

# Concurrency
# I reccommend reading the below article and probably using threads or something like that rather than a separate process
# The above functions could be implemented using a 'each call spawns a thread which then triggers a callback' paradigm
# or one which is more like a single thread that constantly updates an internal log of its position and heading,
# allowing calls from the recon command script to be answered immediatley (this would probably be much better)
# 
# I wouldn't worry much about this for now - if you can get it talking to the Pixhawk reliably, we can discuss this later
#
# https://medium.com/@bfortuner/python-multithreading-vs-multiprocessing-73072ce5600b

# example code used to process images in separate threads (python concurrent.futures module)
executor = concurrent.futures.ThreadPoolExecutor(max_workers=ipWorkerCount)
future = executor.submit(processImage, image)
future.add_done_callback(onImageProcessed)