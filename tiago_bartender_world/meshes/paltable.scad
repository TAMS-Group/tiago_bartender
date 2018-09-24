




cube([800, 2000, 30], true);

module a() { 
    translate([(800 - 680) / -2 + 14, 2000 / 2 - 15 - 2, -730/2]) cube([680, 30, 730], true);
};

a();
mirror([0,1,0]) a();