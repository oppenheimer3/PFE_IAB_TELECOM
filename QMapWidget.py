from PyQt5.QtWebEngineWidgets import QWebEngineView ,QWebEnginePage,QWebEngineSettings
from PyQt5.QtCore import QUrl,pyqtSignal
import os
import json
from math import radians, cos, sin, asin, sqrt
import numpy as np

### distance between 2 points in earth
def haversine(lat1,lon1 , lat2, lon2):
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6371 
    return c * r



i=0


class WebEnginePage(QWebEnginePage): 
    console_signal=pyqtSignal()

    def __init__(self):
        super().__init__()
        self.console_output = None 
        
    def stop_imageload(self):
        self.settings().setAttribute(QWebEngineSettings.AutoLoadImages, False)




        
    def javaScriptConsoleMessage(self, level, msg, line, sourceID):
        try:
            console_dict = json.loads(msg)
            self.console_output=console_dict
            if console_dict['type']=='polygon':
                self.console_signal.emit()
        except: self.console_output=msg

    def javaScriptAlert(self,*args): 
        pass
    def getGetConsole(self):
        return self.console_output

class MapWidget(QWebEngineView):
    def __init__(self):
        super().__init__()
        self.page = WebEnginePage()
        self.setPage(self.page)
        self.build()


    def add_marker(self,pos,icon,angle=0):
        self.page.runJavaScript("""
            var marker = L.marker(
                """+str(pos)+""",
            ).addTo(map_1);
            var icon = L.divIcon({"className": "empty", "html": '<img src="""+ icon +""" alt="Drone" style="width: 50px; height: 50px; transform: translate(-50%, -50%) rotate("""+str(angle)+"""deg);">'});
            marker.setIcon(icon);
        """)

    def get_polyline_coordinates(self):
        return self.page.getGetConsole()['cords']
    
    def polyline_in_polygon(self):    # draw polyline inside polygon
        def get_ab(x1,y1,x2,y2):
            if x1 !=x2:
                a=(y2-y1)/(x2-x1)
                b=y1-a*x1
                return [a,b]
            else:return[y2*(1/x1),0]
        def get_lat(x,p):
            x=np.array(x)
            return x*p[0]+p[1]
        if self.page.getGetConsole()['type'] == 'polygon':
            cords=self.page.getGetConsole()['cords']
            lng_list=[]
            lat_list=[]
            for cord in cords[0]:
                lng_list.append(float(cord['lng']))
                lat_list.append(float(cord['lat']))         
            max_lng=max(lng_list)
            min_lng=min(lng_list)
            min_lng_index=lng_list.index(min_lng)
            lng_list=lng_list[min_lng_index:]+lng_list[:min_lng_index]
            lat_list=lat_list[min_lng_index:]+lat_list[:min_lng_index]
            lng_list=lng_list+[lng_list[0]]
            lat_list=lat_list+[lat_list[0]]
            max_lng_index=lng_list.index(max_lng)
            e=5*1e-5
            x=np.arange(min_lng,max_lng,e)
            upper_points=[]
            lower_points=[]
            for i in range(0,len(cords[0])):
                ab=get_ab(lng_list[i],lat_list[i],lng_list[i+1],lat_list[i+1])
                du=[x for x in x if  lng_list[i]<=x<=lng_list[i+1]]
                upper_points.extend(get_lat(du,ab))
                dl=[x for x in x if  lng_list[i+1]<=x<=lng_list[i]]
                dl.reverse()
                lower_points.extend(get_lat(dl,ab))
            lower_points.reverse()
            polyline_cords=[]
            m=(len(x)-2)%2
            for i in range(0,len(x)-2,2):
                polyline_cords.extend([[upper_points[i],x[i]],[upper_points[i+1],x[i+1]],[lower_points[i+1],x[i+1]],[lower_points[i+2],x[i+2]]])
            polyline_cords.extend([[upper_points[len(x)-2+m],x[len(x)-2+m]],[lat_list[max_lng_index],max_lng]])
            self.page.runJavaScript("""
                    var myPolyline"""+str(i)+""" = L.polyline("""+str(polyline_cords)+ """, {color: 'green'}).addTo(map_1);
                    console.log(JSON.stringify({type:'polyline' , cords:myPolyline"""+str(i)+"""._latlngs}))
            """)
            i+=1
            return True


    def delete_polyline(self):
        self.page.runJavaScript("""
                for(i in map_1._layers) {
                    if(map_1._layers[i]._path != undefined) {
                        try {
                            map_1.removeLayer(map_1._layers[i]);
                        }
                        catch(e) {
                            console.log("problem with " + e + map_1._layers[i]);
                        }
                    }
                }   
        """)
    def center(self,pos):
        self.page.runJavaScript(f"""
            map_1.setView(new L.LatLng({pos[0]},{pos[1]}),20);

        """)

    def update_marker(self,pos,icon,angle):
        self.page.runJavaScript("""
            marker.setLatLng("""+str(pos)+""");
            var icon = L.divIcon({"className": "empty", "html": '<img src="""+ icon +""" alt="Drone" style="width: 50px; height: 50px; transform:translate(-50%, -50%) rotate("""+str(angle)+"""deg);">'});
            marker.setIcon(icon);

        """)

    def distance(self,lat1,lon1,lat2,lon2):
        return haversine(lat1,lon1 , lat2, lon2)


    def add_tile(self,tile,j):
        url=tile['tile_url']
        index=tile['index']
        self.page.runJavaScript("""
            map_1.removeControl(control);
            var new_layer"""+str(j)+""" = L.tileLayer(
                """+f'"{url}"'+""",
                {"attribution": "Google Earth Engine", "detectRetina": false, "maxNativeZoom": 24, "maxZoom": 24, "minZoom": 0, "noWrap": false, "opacity": 1.0, "subdomains": "abc", "tms": false}
            ).addTo(map_1);

            layer_control.overlays['"""+index+str(j)+"""']=new_layer"""+str(j)+""";""")
    
    def update_layer_control(self):
        self.page.runJavaScript("""
            map_1.removeControl(control);
             var control=L.control.layers(
                layer_control.base_layers,
                layer_control.overlays,
                {"autoZIndex": true, "collapsed": true, "position": "topright"}
            ).addTo(map_1);

        """)

    

    def build(self):
        m=open('./static/mymap.html','r')
        self.setHtml(m.read(),baseUrl=QUrl.fromLocalFile(os.getcwd()+os.path.sep))


    def image_overlay(self,img,bounds):
        self.page.runJavaScript("""
        var imageUrl = """+img+""";
        var errorOverlayUrl = 'https://cdn-icons-png.flaticon.com/512/110/110686.png';
        var altText = 'Drone Map';
        var latLngBounds = L.latLngBounds("""+str(bounds)+""");

        var imageOverlay = L.imageOverlay(imageUrl, latLngBounds, {
            opacity: 0.8,
            errorOverlayUrl: errorOverlayUrl,
            alt: altText,
            interactive: true
        }).addTo(map_1);
        
        map_1.setView(new L.LatLng("""+str(bounds[0][0])+""","""+str(bounds[0][1])+"""),20);

        
        
        
        """)