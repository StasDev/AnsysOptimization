f = open(r'C:\Users\1\Desktop\Work\Lopast_helicopter_13_10\Scripts\results.txt', 'a')
db.models['ACP Model'].update(objects=[db.models['ACP Model'].solutions['Solution 1']])
model = db.models['ACP Model']
FC = db.models['ACP Model'].definitions['FailureCriteria.1']
result = model.solutions['Solution 1'].query(definition=FC, position='centroid', selection='all', component='rf',solution_set=-1)
f.write(str(float(result[25])) + '\n')
f.close()