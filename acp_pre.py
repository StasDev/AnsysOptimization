f = open(r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\angles.txt', 'r')
angles = f.read().split('\n')
angles.pop()
f.close()
f = open(r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\results.txt', 'a')
db.models['ACP Model'].material_data.stackups['Stackup.1'].fabrics = [(db.models['ACP Model'].material_data.fabrics['Fabric.1'], float(angle)) for angle in angles]
db.models['ACP Model'].update()
for angle in angles:
    f.write(angle + ',')
f.close()