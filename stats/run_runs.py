import subprocess,time,sqlite3,rospkg,os,imageio
import stats
from pygifsicle import optimize
import matplotlib.pyplot as plt
import moviepy.editor as mp
import numpy as np

def make_gif(ex, db_path, map):
    execution = ex

    SR = stats.SingleRun(db_path, ex)

    fig, ax = plt.subplots(dpi=200)
    x = stats.MAPS[map] #  'office_big', 'office_2', 'map23', 'e13','e40','13-3

    start_time = SR.get_start_time()+2
    frame = 20
    output_path = "figures/gif/ex" + str(execution)
    output_path = output_path.replace(" ", "\\ ")
    print(os.getcwd()+"/"+output_path)
    os.makedirs(os.getcwd()+"/"+output_path, exist_ok=True)
    i=0
    rendez_time = SR.get_rendezvous_time()
    time_iter = np.arange(
        start_time,
        start_time+rendez_time if rendez_time else SR.get_last_time(),
        frame
    )
    imgcounter = 0
    for t in time_iter:
        imgcounter+=1
        stats.plot_map(
            stats.get_map_img(x),
            ax
        )
        SR.plot_tracciato(ax=ax, map=x, time=t)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_title(round(t,2))
        plt.savefig(f'{output_path}/fig{i+1}.png', bbox_inches = "tight")
        ax.set_aspect('equal')
        i+=1
        ax.clear()
        if imgcounter > 500 :
            break

    images = []
    for j in range(len(time_iter)):
        images.append(imageio.imread(f"{output_path}/fig{j+1}.png"))
    imageio.mimsave(f'figures/gif/ex{execution}.gif', images, duration=250)
    optimize(f'figures/gif/ex{execution}.gif', f'figures/gif/ex{execution}.gif')

    clip = mp.VideoFileClip(f'figures/gif/ex{execution}.gif')
    clip.write_videofile(f'figures/gif/ex{execution}.mp4')

if __name__ == '__main__':

    GOAL = 20
    LAUNCHFILE = "test_3_FBE_big_3R.launch"

    SUCCESS = "RENDEZVOUS!"
    ROSLAUNCH = ["roslaunch","journal_rendezvous",LAUNCHFILE]
    TIMEOUT = 10000
    TIME_INFO_RATE = 60*5
    CHECK_REND_RATE = 20
    START_TIMEOUT = 10  #secondi in cui la run deve partire ed inserire le posizioni nel db

    USELESS_MSG = ["Laser Pose=","Registering Scans","Done","[WARN]","Warning:"]

    package_dir = rospkg.RosPack().get_path('journal_rendezvous')
    db_path = package_dir+'/data/data_test.db'
    try:
        conn = sqlite3.connect(db_path, check_same_thread=False)
    except sqlite3.Error as e:
        print(e)

    RENDEZVOUS_NR = 0

    while RENDEZVOUS_NR<GOAL:
        cur = conn.cursor()
        execution_nr = None
        while execution_nr is None:
            cur.execute('SELECT MAX(id) FROM Executions')
            execution_nr = cur.fetchone()[0]+1
        cur.close()

        process = subprocess.Popen(
            ROSLAUNCH,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        time.sleep(4)
        print(f"Partita run {execution_nr}")
        rate_count = 1
        check_rend_count = 1

        broken_run = False
        timeout_run = False

        start_time = time.time()
        while True:
            diff = time.time()-start_time
            cur = conn.cursor()
            cur.execute('SELECT * FROM Clustering WHERE execution=?',(execution_nr,))
            res = cur.fetchall()
            cur.close()
            if len(res)>0: break   #la run è partita
            if diff>START_TIMEOUT: #la run si è rotta all'avvio
                print("\tRUN non partita, riavvio")
                broken_run = True
                break
            time.sleep(0.5)

        start_time = time.time()
        with open("last_run.txt","w") as log:
            while True:
                if broken_run: break
                try:
                    line = process.stdout.readline()
                except UnicodeDecodeError: pass
                diff = time.time()-start_time
                if diff>TIMEOUT: 
                    print("\tTempo scaduto")
                    timeout_run = True
                    break
                if diff>rate_count*TIME_INFO_RATE:
                    print(f"\tLa run è in esecuzione da {round(diff/60)} minuti...")
                    rate_count += 1
                if diff>check_rend_count*CHECK_REND_RATE:
                    cur = conn.cursor()
                    cur.execute('SELECT rendezvous FROM Executions WHERE id=?',(execution_nr,))
                    RENDEZVOUS = cur.fetchone()[0]
                    cur.close()
                    if RENDEZVOUS: 
                        print(f"\t{SUCCESS}")
                        break
                if not any((msg in line for msg in USELESS_MSG)):
                    log.write(line)
                #print(line)
        #time.sleep(10)
        print(f"\tRUN {execution_nr} TERMINATA")
        process.terminate()
        cur = conn.cursor()
        cur.execute('SELECT rendezvous FROM Executions WHERE id=?',(execution_nr,))
        RENDEZVOUS = cur.fetchone()[0]
        if RENDEZVOUS: print(f"Run {execution_nr} corretta")
        cur.close()
        RENDEZVOUS_NR += RENDEZVOUS
        cur = conn.cursor()
        cur.execute('SELECT map FROM Executions WHERE id=?',(execution_nr,))
        map_name = cur.fetchone()[0]
        cur.close()
        time.sleep(10)
        if not (timeout_run or broken_run): make_gif(execution_nr,db_path,map_name)
        time.sleep(60) #tempo di chiudersi tutto
    
