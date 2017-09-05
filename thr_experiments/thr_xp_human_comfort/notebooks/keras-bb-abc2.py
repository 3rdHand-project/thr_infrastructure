
# coding: utf-8

# In[21]:

from keras.models import Sequential, model_from_json
from keras.layers.core import Dense, Activation, Dropout
import numpy as np


# In[2]:

model = Sequential()
model.add(Dense(128, input_dim=18, activation='linear'))
model.add(Dense(256, activation='tanh'))
model.add(Dropout(0.2))
model.add(Dense(256, activation='tanh'))
model.add(Dropout(0.2))
model.add(Dense(256, init='uniform', activation='linear'))
model.add(Dense(1, activation='linear'))
model.compile(loss='mse', optimizer='rmsprop')


# In[3]:

model.summary()


# In[5]:

bmanDataName = "bb-abc2.csv"


# In[6]:

bmanData = np.loadtxt(bmanDataName, delimiter=",", skiprows=1)


# In[11]:

bmanData


# In[7]:

bmanData.shape


# In[9]:

rawData = bmanData[:,1:]
rawLabels = bmanData[:,0]

indices = np.arange(len(rawLabels))
np.random.shuffle(indices)

split = 0.8 # how much of the data is training. split = 0.8 means 80% train, 20% test
splitIdx = int(round(len(rawLabels)*0.8))

X_train = rawData[indices[:splitIdx]]
y_train = rawLabels[indices[:splitIdx]]

X_test = rawData[indices[splitIdx:]]
y_test = rawLabels[indices[splitIdx:]]


# In[10]:

print (X_train.shape)
print (y_train.shape)

print (X_test.shape)
print (y_test.shape)


# In[12]:

epochs = 200
batch_size = 64
val_split = 0.2


# In[ ]:

hist = model.fit(X_train, y_train, nb_epoch=epochs, batch_size=batch_size, validation_split=val_split)


# In[15]:

score = model.evaluate(X_test, y_test, batch_size=batch_size)


# In[16]:

score # this gives the mean squared error over the test set


# In[17]:

modelName = "bb-abc"
modelVersion = 1


# In[24]:

open("model-"+modelName+'-'+str(modelVersion)+'.json', 'w').write(model.to_json())
model.save_weights("model-"+modelName+'-'+str(modelVersion)+'.h5')


# In[25]:

# @Baptiste, if you wanna load the model I trained 
# and play around with it, use those two lines, 
# instead of the block way up above, 
# with the model=Sequential()... and so on

model = model_from_json(open("model-"+modelName+'-'+str(modelVersion)+'.json').read())
model.load_weights("model-"+modelName+'-'+str(modelVersion)+'.h5')


# In[64]:




# In[28]:

print model.predict([X_test[0:1]]) # this uses the model to predict the output for the first element in the test set


# In[29]:

print y_test[0] # this is the actual test set output


# In[ ]:



