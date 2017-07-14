from sklearn.neural_network import MLPRegressor
from matplotlib import pyplot as plt

# create Trainig Dataset
train_x = [[x, x, x] for x in range(200)]
train_y = [[x[0]**2, x[1] ** 1.5, x[2] + 3] for x in train_x]

# create neural net regressor
reg = MLPRegressor(solver="lbfgs")
reg.fit(train_x, train_y)
predict = reg.predict(train_x)

plt.plot(train_x, predict, "xr", label="result")
plt.plot(train_x, train_y, label="Training data")
plt.legend()
plt.grid(True)
plt.show()
