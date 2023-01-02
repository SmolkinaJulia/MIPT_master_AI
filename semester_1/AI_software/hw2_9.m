% task 9 smolkina
%Персистентные переменные локальны для функции,
% в которой они объявляются, все же их значения сохраняются
% в памяти между вызовами функции. 
%global per
setGlobal(9)
per
function[per] = setGlobal(val)
    global per;
    per = val;
end