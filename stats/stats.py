import pandas as pd
import shapely as shp
import numpy as np
import sqlite3, zlib, pickle, base64, json
from matplotlib.colors import ListedColormap, BoundaryNorm
import matplotlib
import matplotlib.pyplot as plt
from math import inf, ceil

rawMaps = pd.read_sql('SELECT * FROM RawMaps', sqlite3.connect('normRawMaps.db'))
rawMaps['map'] = rawMaps['map'].apply(lambda x: pickle.loads(zlib.decompress(x)))
MAPS = {}
for m in rawMaps['name']:
    MAPS[m] = rawMaps[rawMaps['name']==m]['map'].iloc[0]

FINAL_MARKER = {
    'marker': 'p',
    'markersize':5,
    'markeredgecolor':'k'
}
COLOR_LIST = ['#1f77b4','#ff7f0e','#2ca02c','#d62728','brown','black','yellow','purple']

def std(a): return np.std(a,ddof=1)

plt.rcParams['hatch.linewidth'] = .6

DB_PATH = 'C:\\Users\\mauro\\Documents'

class SingleRun:
    def __init__(self, db_path:str, ex:int, real_map:str=None, verbose:bool=False):
        self.ex = ex
        self.verbose = verbose
        self.conn = sqlite3.connect(db_path)
        #self.conn = 'sqlite:////data_test.db'
        self.exec = pd.read_sql(f'SELECT * FROM Executions WHERE id={ex}', self.conn)
        self.robot_nr = self.exec['robot_nr'][0]
        self.rendezvous = bool(self.exec['rendezvous'][0])
        self.where = f'WHERE execution={ex}'
        self.positions = None
        self.clustering = None
        self.exploration = None
        self.real_map = real_map
        self.finalMaps = None
        self.finalMap  = None

    def __getattribute__(self, name):
        if name=='positions': 
            if object.__getattribute__(self,name) is None:
                if self.verbose: print(f'Carico {name}')
                self.positions = pd.read_sql(f'SELECT * FROM Positions {self.where}', self.conn)
                self.positions = self.positions[(self.positions['x']!=0) & (self.positions['y']!=0)]
        elif name=='clustering':
            if object.__getattribute__(self,name) is None:
                if self.verbose: print(f'Carico {name}')
                self.clustering = pd.read_sql(f'SELECT * FROM Clustering {self.where}', self.conn)
        elif name=='exploration':
            if object.__getattribute__(self,name) is None:
                if self.verbose: print(f'Carico {name}')
                self.exploration = pd.read_sql(f'SELECT * FROM Exploration {self.where}', self.conn)
        elif name=='finalMaps':
            if object.__getattribute__(self,name) is None:
                if self.verbose: print(f'Carico {name}')
                self.finalMaps = pd.read_sql(f'SELECT * FROM FinalMaps {self.where}', self.conn).sort_values('robot')
                self.finalMaps['map'] = self.finalMaps['map'].apply(decode_map)
        elif name=='finalMap':
            if object.__getattribute__(self,name) is None:
                if self.verbose: print(f'Carico {name}')
                self.finalMap = self.get_final_map()
        return object.__getattribute__(self,name)

    def __setattr__(self, name, value):
        if name=='real_map':
            if value not in MAPS and value is not None:
                raise Exception(f'Nome della mappa non valido.\n Valori validi: {list(MAPS.keys())}')
        object.__setattr__(self,name,value)

    def __repr__(self):
        return f'Esecuzione {self.ex}: {self.get_robot_nr()} robot'

    def get_robot_nr(self):
        return self.robot_nr

    def get_tracciato(self, robot, time=999999, origin=(0,0), resolution=1):
        if resolution<=0:
            print('Risolzione <= 0')
        posizioni = self.positions[(self.positions['robot']==robot)]
        posizioni = posizioni[posizioni['time']<time]
        posizioni = posizioni[(posizioni['x']!=0)&(posizioni['y']!=0)]
        o_x, o_y = origin
        lista_posizioni_xy = (((posizioni['x']-o_x)/resolution).to_list(), ((posizioni['y']-o_y)/resolution).to_list())
        return shp.LineString(
            [(lista_posizioni_xy[0][i],lista_posizioni_xy[1][i]) for i in range(len(lista_posizioni_xy[0]))]
        )

    def get_last_pos(self,robot,time=9999999):
        last_pos_tot = self.positions[
                (self.positions['robot']==robot)&(self.positions['time']<time)
            ].sort_values('time').iloc[-1]
        return (last_pos_tot['x'], last_pos_tot['y'])

    def get_start_time(self):
        return self.clustering['time'].min() #-5.746843665768201 #valore ottenuto da analisi su db

    def get_first_pos(self,robot):
        first_pos_tot = self.positions[self.positions['robot']==robot].sort_values('time').iloc[0]
        return (first_pos_tot['x'], first_pos_tot['y'])

    def plot_tracciato(self, time=999999, final_marker=FINAL_MARKER, colors=COLOR_LIST, 
                       ax=None, map=None, legend=False, scia=True, information_decay=False, 
                       id_frontiers=False, id_radius=2.7, id_time=333):
        if ax is None:
            ax=plt.gca()
        if map is not None:
            o_x,o_y = (map['info']['origin']['position']['x'], map['info']['origin']['position']['y'])
            resolution = map['info']['resolution']
            #plot_map(map, ax)
        else:
            o_x,o_y = 0,0
            resolution = 1
        robot = [i for i in range(self.get_robot_nr())]
        if scia:
            for r in robot:
                tracciato = self.get_tracciato(robot=r+1, origin=(o_x,o_y), resolution=resolution,time=time)
                ax.plot(*tracciato.coords.xy, color=colors[r%len(colors)], linewidth=.8)
        for r in robot:
            color = colors[r%len(colors)]
            last_pos_x, last_pos_y = self.get_last_pos(r+1,time)
            last_pos = ((last_pos_x-o_x)/resolution,(last_pos_y-o_y)/resolution)
            first_pos_x, first_pos_y = self.get_first_pos(r+1)
            first_pos = ((first_pos_x-o_x)/resolution, (first_pos_y-o_y)/resolution)
            if not shp.Point(first_pos).distance(shp.Point(last_pos))<.3:
                ax.plot(*last_pos, color=color, **final_marker)
            if scia:
                ax.plot(*first_pos, 'o', color=color, markersize=5, markeredgecolor='k', label=f'$r_{r+1}$')
            if information_decay: #non gestisce unioni di più di 2 robot
                pos = self.positions
                id_poses = pos[(pos['robot']==r+1)&(pos['time']>time-id_time)&(pos['time']<time)]
                id_poses_xy = (((id_poses['x']-o_x)/resolution).to_list(), ((id_poses['y']-o_y)/resolution).to_list())
                id_trace = shp.LineString(
                    [(id_poses_xy[0][i],id_poses_xy[1][i]) for i in range(len(id_poses_xy[0]))]
                ).buffer(id_radius/resolution)
                trace = self.get_tracciato(
                    r+1, origin=(o_x,o_y), resolution=resolution,time=time
                ).buffer(id_radius/resolution)
                forgotten_trace = trace.difference(id_trace)
                #print(pos)
                options = {'color':color, 'linewidth':.6, 'linestyle':'--'}
                fill_options = {'color':color, 'alpha':.2}
                if type(forgotten_trace) is shp.Polygon:
                    #ax.fill(*forgotten_trace.exterior.xy,**fill_options)
                    ax.fill(*forgotten_trace.exterior.xy,facecolor="none",edgecolor=color,linewidth=0,hatch='..')
                    ax.plot(*forgotten_trace.exterior.xy,**options)
                else:
                    for g in forgotten_trace.geoms:
                        #ax.fill(*g.exterior.xy,**fill_options)
                        ax.fill(*g.exterior.xy,facecolor="none",edgecolor=color,linewidth=0,hatch='..')
                        ax.plot(*g.exterior.xy,**options)
                if type(id_trace) is shp.Polygon:
                    ax.fill(*id_trace.exterior.xy,**fill_options)
                    ax.plot(*id_trace.exterior.xy,**options)
                else:
                    for g in id_trace.geoms:
                        ax.fill(*g.exterior.xy,**fill_options)
                        ax.plot(*g.exterior.xy,**options)
                #if id_frontiers: #DIFFICILE :(
            if legend:
                ax.legend()

    def get_last_time(self):
        return self.positions['time'].max()

    def is_rendezvous(self):
        return self.rendezvous

    def get_rendezvous_time(self):
        n = self.get_robot_nr()
        tmp = self.clustering[self.clustering['max']==n]['time']
        if not self.is_rendezvous():
            return None
        last_time = tmp.iloc[0]
        return last_time - self.get_start_time()
    
    def get_max_cluster(self):
        res = self.clustering[['time','max']].groupby('max',as_index=False).first()
        max_robot = self.clustering['max'].max()
        for n in range(max_robot,0,-1):
            if len(res[res['max']==n]['time'])==0:
                res = pd.concat(
                    [res,pd.DataFrame(
                        {'max':[n],'time':[res[res['max']==n+1]['time'].iloc[0]]}
                    )]
                )
        res['time'] = res['time'].apply(lambda x:x-self.get_start_time())
        return res.sort_values('max')
    
    def get_final_map_single(self, robot):
        fm = self.finalMaps[self.finalMaps['robot']==robot].copy()
        if len(fm)==0:
            return None
        return fm.iloc[-1]
    
    def get_final_map(self, old_method=False):
        if any([self.get_final_map_single(n) is None for n in range(1,self.get_robot_nr()+1)]):
            return None
        maps = [self.get_final_map_single(n)['map'] for n in range(1,self.get_robot_nr()+1)]
        if old_method:
            return old_merge_maps(maps,real_map=MAPS[self.real_map] if self.real_map else None) 
        return merge_maps(maps,real_map=MAPS[self.real_map] if self.real_map else None)

    def check_final_maps(self):
        n = self.robot_nr
        _, axs = plt.subplots(ceil((n+1)/2), 2, dpi=200)
        borders = []
        for robot in range(1, n+1):
            ax = axs[(robot-1)//2][(robot-1)%2]
            map = self.get_final_map_single(robot)['map']
            plot_occ_grid(map, ax)
            map_img = get_map_img(map)
            min_x = -1
            min_y = -1
            max_x = -1
            max_y = -1
            h, w = len(map_img), len(map_img[0]) 
            res = get_res(self.finalMap)
            for c in range(w):
                if min_x!=-1 and max_x!=-1: break
                if min_x==-1 and (map_img[:,c:c+1]!=-1).any():     min_x = c
                if max_x==-1 and (map_img[:,w-c-1:w-c]!=-1).any(): max_x = (w-c-1)
            for r in range(h):
                if min_y!=-1 and max_y!=-1: break
                if min_y==-1 and (map_img[r:r+1,:]!=-1).any():     min_y = r
                if max_y==-1 and (map_img[h-r:h-r+1,:]!=-1).any(): max_y = (h-r)
            borders.append((min_x,min_y,max_x,max_y))
            trace = self.get_tracciato(
                robot,
                origin = (get_origin(map)['x'], get_origin(map)['y']),
                resolution = get_res(map)
            )
            ax.plot(
                *trace.coords.xy,
                linewidth=.6,
                color=COLOR_LIST[(robot-1)%len(COLOR_LIST)]
            )
            ax.plot(
                [min_x,max_x,max_x,min_x,min_x],
                [min_y,min_y,max_y,max_y,min_y],
                linewidth=.6,
                color=COLOR_LIST[(robot-1)%len(COLOR_LIST)]
            )    
            ax.set_title(f'robot {robot}')
            ax.axis('off')
        plot_occ_grid(
            self.finalMap,
            axs[-1][-1]
        )
        marker = {
            'marker': 'p',
            'markersize':4,
            'markeredgecolor':'k'
        }
        self.plot_tracciato(ax=axs[-1][-1], map=self.finalMap, final_marker=marker)
        res = get_res(self.finalMap)
        m_ox, m_oy = (get_origin(self.finalMap)['x'], get_origin(self.finalMap)['y'])
        for robot in range(1, n+1):
            o_x = get_origin(self.get_final_map_single(robot)['map'])['x']
            o_y = get_origin(self.get_final_map_single(robot)['map'])['y']
            min_x = borders[robot-1][0]-((m_ox-o_x)/res)
            min_y = borders[robot-1][1]-((m_oy-o_y)/res)
            max_x = borders[robot-1][2]-((m_ox-o_x)/res)
            max_y = borders[robot-1][3]-((m_oy-o_y)/res)
            axs[-1][-1].plot(
                [min_x,max_x,max_x,min_x,min_x],
                [min_y,min_y,max_y,max_y,min_y],
                linewidth=.6,
                color=COLOR_LIST[(robot-1)%len(COLOR_LIST)]
            )
        axs[-1][-1].axis('off')
        axs[-1][-1].set_title('merged')
        plt.subplots_adjust(wspace=0, hspace=0.15)

    def get_perc_area(self, check_map=False, check_pos=False):
        if self.verbose: print(self.ex)
        if self.real_map is None:
            raise Exception("Valore della mappa reale richiesto, modificare l'attributo real_map")
        if not self.is_rendezvous():
            return 1
        if self.finalMap is None:
            return None
        if check_map:
            plot_map(get_map_img(self.finalMap), plt.gca())
        if check_pos:
            self.plot_tracciato(ax=plt.gca(), map=self.finalMap)
        explored_img = get_map_img(self.finalMap)
        explored = len(explored_img[explored_img==0])
        real_img = get_map_img(MAPS[self.real_map])
        real = len(real_img[real_img==0])
        return explored/real

class TestSet:
    def __init__(self, db_path:str, test_set:dict, map_name:str, verbose:bool=False):
        ''' test_set = {
            'method1': [ex1,...,exn],
            ...,
            'methodm': [ex1,...,exn]
            } '''
        self.test_set = test_set
        if map_name not in MAPS:
            raise Exception(f'Nome della mappa non valido.\n Valori validi: {list(MAPS.keys())}')
        self.map_name = map_name
        self.methods = list(test_set.keys())
        self.data = {method:[SingleRun(db_path,ts,real_map=map_name,verbose=verbose) for ts in test_set[method]] for method in test_set}
        if any([len(set([sr.get_robot_nr() for sr in self.data[meth]]))!=1 for meth in self.data]):
            raise Exception('Le run non hanno lo stesso numero di robot')
        self.robot_nr = self.data[self.methods[0]][0].get_robot_nr()
        
    def get_rendezvous_times(self):
        return {meth:[sr.get_rendezvous_time() for sr in self.data[meth]] for meth in self.test_set}
    
    def get_rendezvous_times_stats(self, aggregate_func):
        return {meth:aggregate_func(values) for meth,values in self.get_rendezvous_times().items()}
    
    def get_max_cluster(self, norm=False):
        res = {meth:[sr.get_max_cluster() for sr in values] for meth,values in self.data.items()}
        max_time = 1
        if norm:
            max_time = max([
                max([x[x['max']==self.robot_nr]['time'].iloc[0] for x in self.get_max_cluster()[meth] if len(x[x['max']==self.robot_nr])>0]) 
                for meth in self.methods
            ])
        for meth in res:
            for single in res[meth]:
                single['time'] = single['time'].apply(lambda x: x/max_time)
        return res
    
    def get_max_cluster_stats(self, aggregate_func, norm=False):
        res = {meth:[] for meth in self.methods}
        data = self.get_max_cluster(norm=norm)
        for meth in self.methods:
            for n in range(1,self.robot_nr+1):
                res[meth]+=[
                    (n, 
                     aggregate_func([df[df['max']==n]['time'].iloc[0] for df in data[meth] if len(df[df['max']==n])!=0])
                    )
                ]
        return res
    
    def get_perc_areas(self):
        return {meth:[sr.get_perc_area() for sr in self.data[meth]] for meth in self.test_set}

    def get_perc_areas_stats(self, aggregate_func):
        res = self.get_perc_areas()
        return {method:aggregate_func(res[method]) for method in res}

    def __repr__(self):
        return f'{self.data}'

def decode_map(zip):
    return pickle.loads(zlib.decompress(base64.b64decode(zip)))

def get_map_img(map, padding=30, cut=False):
    #plot_map(map, detailed=True)
    height = map['info']['height']
    width = map['info']['width']
    img = np.reshape(map['data'],[height,width])
    if cut:
        min_x = width
        max_x = 0
        min_y = height
        max_y = 0
        for i in range(height):
            for j in range(width):
                if img[i][j]>70 and j<min_x:
                    min_x = j
                if img[i][width-j-1]>70 and width-j-1>max_x:
                    max_x = width-j-1
        for j in range(width):
            for i in range(height):
                if img[i][j]>70 and i<min_y:
                    min_y = i
                if img[height-i-1][j]>70 and height-i-1>max_y:
                    max_y = height-i-1
        if min_y<padding:
            min_y=padding
        if min_x<padding:
            min_x=padding
        cut_img = img[min_y-padding:max_y+padding,min_x-padding:max_x+padding]
    else:
        cut_img = img
    return cut_img

def plot_occ_grid(occ_grid, ax):
    plot_map(get_map_img(occ_grid),ax)

def plot_map(img, ax, bg_color="#607372"):
    cmap = ListedColormap(['#BDC1BC','black','#E000FF'])
    bounds = [-0.5,0.5,100.5,200]
    norm = BoundaryNorm(bounds, cmap.N)
    cmap.set_bad(color=bg_color)
    masked = np.ma.masked_where(img==-1,img)
    ax.imshow(masked,cmap=cmap,norm=norm,origin='lower')

get_size   = lambda map: (map['info']['height'], map['info']['width'])
get_origin = lambda map: map['info']['origin']['position']
get_res    = lambda map: map['info']['resolution']

def merge_maps(maps, real_map=None, threshold=70):
    if real_map: maps = [real_map] + maps + [real_map]  
    min_x = min([get_origin(map)['x'] for map in maps])
    min_y = min([get_origin(map)['y'] for map in maps])
    max_x = max([get_origin(map)['x']+get_size(map)[1]*get_res(map) for map in maps])
    max_y = max([get_origin(map)['y']+get_size(map)[0]*get_res(map) for map in maps])
    h = round((max_y-min_y)/get_res(maps[0]))
    w = round((max_x-min_x)/get_res(maps[0]))
    img = np.zeros((h,w),dtype=np.int16)-1
    for idx, map in enumerate(maps):
        res     = get_res(map) 
        left    = round((get_origin(map)['x'] - min_x)/res)
        right   = left + get_size(map)[1]
        bottom  = round((get_origin(map)['y'] - min_y)/res)
        up      = bottom + get_size(map)[0]
        section = img[bottom:up,left:right]
        map_img = get_map_img(map)
        if real_map and idx==len(maps)-1:
            img[bottom:up,left:right] = section + map_img
            section[section==-2] = -1
            section[section==99] = 100
        else:
            img[bottom:up,left:right] = section * map_img
            if real_map and idx==0:
                img[img==0] = 100
            img[bottom:up,left:right] = section * -1
        section[section>100] = 100
        section[section<-1]  = 100
    merged_map = {
        'info': { 
            'width': w,
            'height': h,
            'resolution': res,
            'origin': {
                'position': {
                    'x': min_x,
                    'y': min_y
                }
            }
        },
        'data': img.flatten()
    }
    return merged_map

def old_merge_maps(maps, real_map=None, threshold=70):
    """
    maps è una lista di occ.grid

    real_map è la mappa completa dell'ambiente occ.grid

    restituisce una mappa in formato occ.grid con solo i campi necessari alla visualizzazione
    """
    grid = {}
    min_x = inf 
    min_y = inf
    max_x = -inf
    max_y = -inf
    for idx, map in enumerate([real_map]+maps):
        if idx==0 and real_map is None: continue
        occupancy = map['data']
        resolution = map['info']['resolution']
        origin_x = map['info']['origin']['position']['x']
        origin_y = map['info']['origin']['position']['y']
        width =  map['info']['width']
        height = map['info']['height']
        for i in range(width):
            for j in range(height):
                pixel = occupancy[i+j * width]
                x = round((i * resolution) + origin_x,2)
                y = round((j * resolution) + origin_y,2)
                p = (x, y)
                if pixel != -1:
                    if idx==0 or pixel>threshold:
                        grid[p] = True
                    elif idx>0 and pixel<=threshold:
                        if real_map is None:
                            grid[p] = False
                        else:
                            if p in grid:
                                grid[p] = False
                if x<min_x:
                    min_x=x
                elif x>max_x:
                    max_x=x
                if y<min_y:
                    min_y=y
                elif y>max_y:
                    max_y=y
    h,w = round(abs(max_y-min_y)/resolution),round(abs(max_x-min_x)/resolution)
    o_x,o_y = min_x,min_y
    img = np.zeros((h+1,w+1),dtype=np.int16)-1
    for p in grid:
        x,y = p
        x,y = round((x-o_x)/resolution),round((y-o_y)/resolution)
        img[y,x] = 100 if grid[p] else 0
    merged_map = {
        'info': { 
            'width': w+1,
            'height': h+1,
            'resolution': resolution,
            'origin': {
                'position': {
                    'x': o_x,
                    'y': o_y
                }
            }
        },
        'data': img.flatten()
    }
    return merged_map

def plot_cluster_max(test_set, agg_func, ax, norm=False, alpha=0.3, add_yticks=[], no_yticks=[]):
    if len([i for i in add_yticks if i>test_set.robot_nr])>0:
        raise Exception('add_yticks errato')
    TS = test_set
    x = TS.get_max_cluster_stats(agg_func, norm=norm)
    stds = TS.get_max_cluster_stats(std, norm=norm)
    ax.plot(
        [i[0] for i in x['my']],
        [i[1] for i in x['my']],
        marker='o',
        label='FBR',
        color='C0'
    )
    ax.plot(
        [i[0] for i in x['el']],
        [i[1] for i in x['el']],
        marker='o',
        label='FBE',
        color='C1'
    )
    ax.fill_between(
        list(range(1,TS.robot_nr+1)),
        [x['my'][i][1]-stds['my'][i][1] for i in range(len(x['my']))],
        [x['my'][i][1]+stds['my'][i][1] for i in range(len(x['my']))],
        alpha=alpha,
        linewidth=0,
        color='C0'
    )
    ax.fill_between(
        list(range(1,TS.robot_nr+1)),
        [x['el'][i][1]-stds['el'][i][1] for i in range(len(x['el']))],
        [x['el'][i][1]+stds['el'][i][1] for i in range(len(x['el']))],
        alpha=alpha,
        linewidth=0,
        color='C1'
    )
    ax.grid()
    ax.set_xticks(range(1,TS.robot_nr+1))
    ax.set_yticks(
        [max([x['my'][i][1],x['el'][i][1]]) for i in range(TS.robot_nr) if i+1 not in no_yticks]
        +
        [min([x['my'][i-1][1],x['el'][i-1][1]]) for i in add_yticks]
    )
    ax.legend(loc='upper left')
    #ax.set_xlabel('$\max{(\left| C \\right|)}$',fontsize=17)
    ax.set_title('$\\max{(\\left| C \\right|)}$')
    ax.set_ylabel('$t$',fontsize=18,rotation='horizontal',labelpad=15)
