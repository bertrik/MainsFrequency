class Regression {

  private:
    double sum_x;
    double sum_y;
    double sum_xx;
    double sum_xy;
    int n;

    double a = 0.0;             // slope
    double b = 0.0;             // intercept on y-axis
    double c = 0.0;             // intercept on x-axis

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
