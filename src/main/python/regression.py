#%%
#
# To run:
#   Create a python venv  (vscode ctl-sh-P Python Create Env)
#   From Terminal started in vscode:
#     pip install pandas plotly scikit-learn
#
#
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

#Can get any degree equation by changing degree
model = make_pipeline(PolynomialFeatures(degree=3), LinearRegression())
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
weights = linear_regression_model.coef_
variableOrder = model.steps[0][1].get_feature_names_out()

print(f"Intercept: {intercept}  ")
print(f"Weights: {weights}  ")
print(f"Variable to weights: {variableOrder}  ")

#print(f"General Equation: RPM = " + round(intercept,2).astype(str) + " + " + round(weights[1],2).astype(str) + " * X + " + round(weights[2],2).astype(str) + " * Y + " + round(weights[3],2).astype(str) + " * X^2 + " + round(weights[4],2).astype(str) + " * X * Y + " + round(weights[5],2).astype(str) + " * Y^2")
print(f"General Equation: Z = " + round(intercept,2).astype(str), end=" ")
for i in range(len(variableOrder) -1):
    print("+ "+round(weights[i+1],2).astype(str) + " * " + variableOrder[i+1] , end=" ")


# %%
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