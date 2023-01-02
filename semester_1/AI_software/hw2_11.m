%hw2-11
rate = [1, 2];
price =[100, 30];
total = income(rate,price);

function week_income = income(rate,price)
    week_income = 0;
    for i=1:length(rate)
         product_income_week = rate(i)*price(i)*8*2*6;
         %smena_income = smena_income + rate(i)*price(j)*8;
         %day_income = day_income + smena_income*2;
         %week_income = week_income + day_income*6;
         week_income = week_income + product_income_week;
    end
end

