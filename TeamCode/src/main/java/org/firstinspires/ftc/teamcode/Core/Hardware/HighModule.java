/*
 * Copyright (c) 2020-2025 High Five (http://www.highfive.ro)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit others to do the same.
 *
 * This permission is granted under the condition that the above copyright notice
 * and this permission notice are included in all copies or significant portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED AS IS, WITHOUT ANY WARRANTIES OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 * THIS INCLUDES, BUT IS NOT LIMITED TO, WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE,
 * OR NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR THE COPYRIGHT HOLDERS BE HELD LIABLE FOR ANY CLAIMS, DAMAGES,
 * OR OTHER LIABILITIES THAT MAY ARISE FROM THE USE OF THE SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Core.Hardware;

public interface HighModule{

    /**
     *
     * @param target
     */
    public void setTarget(double target);

    /**
     *
     * @param target
     * @param time
     */
    public void setTarget(double target, double time);

    public boolean atTarget();

    /**
     *
     * @return
     */
    public double getTarget();

    /**
     *
     */
    public void update();
}
