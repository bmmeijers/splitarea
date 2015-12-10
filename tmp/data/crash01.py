from simplegeom.geometry import LineString
import json
from pprint import pprint
with open('crash01.geojson') as fh:
    c = json.loads(fh.read())

for feature in c['features']: 
    heading = [value for value in feature['properties'].iterkeys()]
    heading.append('geometry')
    print ";".join(map(str, heading))
    break

# select the edges that we need for the test case!
ext = [1010199, 1004953, 1010646]
needed = [1010852, 1010437, 1009934, 1005389, 1903]

external = []
polygon = []
for feature in c['features']: 
    props =  [feature['properties']['edgeId'], feature['properties']['startNodeId'], feature['properties']['endNodeId'], feature['properties']['leftFaceId'], feature['properties']['rightFaceId']]
    props.append(LineString(feature['geometry']['coordinates']))
    edgeId = feature['properties']['edgeId']
    if edgeId in ext:
        external.append(props)
#        print ";".join(map(str, props))
    if edgeId in needed:
        polygon.append(props)
#        print ";".join(map(str, props))

print "external = \\"
pprint(external)
print "polygon = \\"
pprint(polygon)
