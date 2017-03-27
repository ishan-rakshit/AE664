function new_unit = unit_conversion(old_unit,opt)
    if strcmp(opt,'power') == 1
        new_unit = old_unit/746;
    end
end