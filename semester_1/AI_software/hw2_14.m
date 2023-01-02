%hw2_14
znak = '/';
a = 3;
b = 6;
rez = math(a,znak,b);
function answer = math(a,znak,b)
switch znak
    case '/'
        answer = a/b;
    case '*'
        answer = a*b;
    case  '+'
        answer = a+b;
    case '-'
        answer = a-b;
end
end
