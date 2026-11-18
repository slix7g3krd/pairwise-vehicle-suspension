"""第 51 批：越野车连续过搓板路时轮子陷地、悬挂爆炸、车身抖：..."""

def collision_mask(is_static, is_dynamic):
    """返回轮子射线应包含的碰撞层。"""
    raise NotImplementedError("TODO: collision_mask")

def integrate_steps(dt, fixed_dt):
    """按固定步长切分悬挂积分步数。"""
    raise NotImplementedError("TODO: integrate_steps")

def anti_roll(left_compression, right_compression):
    """返回防倾杆力矩方向。"""
    raise NotImplementedError("TODO: anti_roll")

def friction_circle(long_force, lat_force, max_force):
    """把轮胎力约束在摩擦圆内。"""
    raise NotImplementedError("TODO: friction_circle")

def diff_lock(speed_delta, threshold):
    """判断差速器是否锁止。"""
    raise NotImplementedError("TODO: diff_lock")

def gravity_slope(gravity, slope_normal):
    """把重力投影到悬挂轴。"""
    raise NotImplementedError("TODO: gravity_slope")

def continuous_ray(start, end, thickness):
    """判断高速轮子射线是否命中薄板。"""
    raise NotImplementedError("TODO: continuous_ray")

def resample_input(samples, target_dt):
    """按固定步长重采样回放输入。"""
    raise NotImplementedError("TODO: resample_input")
