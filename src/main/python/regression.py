#%%
import numpy as np
import pandas as pd
import plotly.graph_objs as go
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

data = pd.read_csv("Position.csv")

X1 = data['Radius'] * np.cos(np.radians(data['Degrees']))
X2 = data['Radius'] * np.sin(np.radians(data['Degrees']))
y = data['RPM']

model = make_pipeline(PolynomialFeatures(degree=2), LinearRegression())
model.fit(np.vstack((X1, X2)).T, y)
linear_regression_model = model.named_steps['linearregression']
scatter_trace = go.Scatter3d(
    x=X1,
    y=X2,
    z=y,
    mode='markers',
    marker=dict(
        size=5,
        color=y,                
        colorscale='Viridis',   
        opacity=0.8
    ),
    text='RPM: ' + y.astype(str),
    name='Data Points'
)

x1_range = np.linspace(min(X1), max(X1), 100)
x2_range = np.linspace(min(X2), max(X2), 100)
X1_surf, X2_surf = np.meshgrid(x1_range, x2_range)
X_surf = np.array([X1_surf.flatten(), X2_surf.flatten()]).T
Y_surf = model.predict(X_surf).reshape(X1_surf.shape)


surface_trace = go.Surface(x=X1_surf, y=X2_surf, z=Y_surf)


layout = go.Layout(
    scene=dict(
        xaxis=dict(title='X Coordinate'),
        yaxis=dict(title='Y Coordinate'),
        zaxis=dict(title='RPM')
    ),
    margin=dict(l=0, r=0, b=0, t=0)
)


fig = go.Figure(data=[scatter_trace, surface_trace], layout=layout)
fig.show()
intercept = linear_regression_model.intercept_
print(f"Intercept: {intercept}  ")
weights = linear_regression_model.coef_
print(f"Weights: {weights}  ")

print(f"General Equation: RPM = " + round(intercept,2).astype(str) + " + " + round(weights[1],2).astype(str) + " * X + " + round(weights[2],2).astype(str) + " * Y + " + round(weights[3],2).astype(str) + " * X^2 + " + round(weights[4],2).astype(str) + " * X * Y + " + round(weights[5],2).astype(str) + " * Y^2")



#test
X1 = 1.496
X2 = 0.105
gen_equation_predicted = intercept + weights[1] * X1 + weights[2] * X2 + weights[3] * X1 * X1 + weights[4] * X1 * X2 + weights[5] * X2 * X2
print(f"General Equation Predicted: {gen_equation_predicted}  ")

X1 = 1
X2 = 0.1
gen_equation_predicted = intercept + weights[1] * X1 + weights[2] * X2 + weights[3] * X1 * X1 + weights[4] * X1 * X2 + weights[5] * X2 * X2
print(f"General Equation Predicted: {gen_equation_predicted}  ")

X1 = 2
X2 = 0.2
gen_equation_predicted = intercept + weights[1] * X1 + weights[2] * X2 + weights[3] * X1 * X1 + weights[4] * X1 * X2 + weights[5] * X2 * X2
print(f"General Equation Predicted: {gen_equation_predicted}  ")
