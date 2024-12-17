import re
from shapely import Point, Polygon, MultiPolygon
import matplotlib.pyplot as plt

class Area:
    def __init__(self, id):
        self.id = id
        self.points = []
        self.polygon = Polygon()
    
    def add_point(self, point):
        if not isinstance(point, Point):
            raise Exception("You can't add a non Point object to an Area")
        self.points.append(point)
        if len(self.points)>2:
            self.polygon = Polygon(self.points)
    
    def add_points(self, points):
        for point in points: self.add_point(point)

    def __iter__(self):
        return self.points.__iter__()
    
    def __repr__(self):
        S = f'Area {self.id}: ['
        if len(self.points)==0:
            return S+'];'
        count = 0
        for point in self.points:
            if count%5==0: S += '\n   '
            S += f'({point.x}, {point.y}), '
            count += 1
        S = S[:-1]
        return S+'\n];'

def plain_text(text):
    return text.replace('\n','').replace(' ','')

def parse_coords(text):
    areas = []
    plain = plain_text(text)
    for subtext in plain.split(";")[:-1]:
        if not subtext.startswith('Area'):
            raise Exception("Wrong coordinates format")
        A = Area(int(re.match('Area[0-9]+', subtext)[0].split('Area')[1]))
        coords = re.findall(
            "\\(-?[0-9]+[\\.[0-9]+]?,-?[0-9]+[\\.[0-9]+]?\\)",
            re.split("Area[0-9]+:", subtext)[1][1:-1]
        )
        points = [Point(*[float(x) for x in c[1:-1].split(',')]) for c in coords]
        A.add_points(points)
        areas.append(A)
    return areas

def parse_from_file(file_path):
    with open(file_path) as file:
        coords = parse_coords(file.read())
    return coords

def poly_union(areas_list, tot=Polygon()):
    if areas_list==[]: return tot
    return poly_union(
        areas_list[:-1],
        tot.union(areas_list[-1].polygon)
    )

def get_exteriors(figure):
    if isinstance(figure, Polygon):
        return [figure.exterior.coords]
    if isinstance(figure, MultiPolygon):
        return [list(fig.exterior.coords) for fig in figure.geoms]

if __name__ == '__main__':
    res = parse_from_file('semantic_coords/marty.coords')
    print(res)
    print(res[0].polygon.exterior.coords.xy)