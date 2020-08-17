import json
import numpy as np
import matplotlib.pyplot as plt
from baselines.common import plot_util as pu

exp_id='payload_empty_her100_demo'
results = pu.load_results('../policies/{}'.format(exp_id))[0]
epoch = results.progress["epoch"]
trainscs = results.progress["train/success_rate"]
testscs = results.progress["test/success_rate"]

plt.plot(epoch,pu.smooth(trainscs,radius=1),label="Train")
plt.plot(epoch,pu.smooth(testscs,radius=1),label="Test")
# plt.plot(epoch,trainscs,label="Train")
# plt.plot(epoch,testscs,label="Test")
plt.xlabel('Epoch')
plt.ylabel('Success Rate')

f = open('../policies/{}/params.json'.format(exp_id))
params = json.load(f)
plt.title(params['env_name'])
plt.legend()
#plt.show()
#print(results.progress)
plt.savefig('plots/exp_{}_results.png'.format(exp_id))
