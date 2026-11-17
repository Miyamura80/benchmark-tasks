// Rim-relative section of the earlier toilet scene, radii and Z in mm.
// X runs rearward and Y runs toward the bowl's left side.
profile = [
    [58,49,-81,0], [64,55,-71,0], [96,74,-58,0], [119,94,-39,0],
    [142,108,-18,0], [154,118,-10,0], [147,111,2,0], [164,128,6,0],
    [182,144,4,0], [191,151,-4,0], [193,152,-17,0], [185,144,-30,4],
    [169,127,-43,10], [145,106,-58,21], [113,83,-76,45], [105,78,-93,75],
    [108,78,-107,80], [119,81,-117,82], [133,89,-129,80], [136,90,-133,79]
];
segments = 96;
count = len(profile);
points = [for (row = profile, i = [0:segments-1])
    [row[3]+row[0]*cos(360*i/segments), row[1]*sin(360*i/segments), row[2]]];
faces = [for (j = [0:count-1], i = [0:segments-1])
    [j*segments+i, j*segments+(i+1)%segments,
     ((j+1)%count)*segments+(i+1)%segments, ((j+1)%count)*segments+i]];

polyhedron(points=points, faces=faces, convexity=8);
