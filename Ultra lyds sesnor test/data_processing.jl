using LinearAlgebra
using DataFrames
using Statistics
using Plots
using CSV

data = CSV.read("sonar_data.csv",DataFrame,header=false)[:,1]

plot(data)

data_20cm = data[1:150]
data_40cm = data[250:400]
data_60cm = data[470:640]
data_80cm = data[810:870]
data_100cm = data[1025:1150]
data_120cm = data[1250:end-80]

all_data = [data_20cm,data_40cm,data_60cm,data_80cm,data_100cm,data_120cm]
expected = [20,40,60,80,100,120]

acc = [mean(dataset)-expect for (dataset,expect) in zip(all_data,expected)]
pre = [std(dataset) for dataset in all_data]

plot(pre)
ylims!(0,1)
plot!(acc)