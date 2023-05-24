class Regression {

  private:
    double sum_x { 0 };
    double sum_y { 0 };
    double sum_xx { 0 };
    double sum_xy { 0 };
    int n { 0 };

    double a { 0.0 };             // slope
    double b { 0.0 };             // intercept on y-axis
    double c { 0.0 };             // intercept on x-axis

  public:
    /**
     * Constructor.
     */
     explicit Regression(void);

    void reset(void);
    void add(double x, double y);

    // calculates the regression, results are available through getA(), getB(), getC()
    void calculate(void);

    double getA();
    double getB();
    double getC();
};
