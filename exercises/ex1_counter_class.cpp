class Counter {
    private:
        double timer;
        int init(void);
        double counter_step(void);

    public:
        Counter() : timer(0) {}
        int inc(double number);
        int dec(double number);
        int value() const {
            return timer;
        }
};

int Counter::init(void) {
    this->timer = 0.0;
    return 0;
}

double Counter::counter_step(void) {
    this->timer += 1.0;
    return this->timer;
}

int Counter::inc(double number){
    this->timer += number;
    return 0;
}

int Counter::dec(double number){
    this->timer -= number;
    return 0;
}

int main(void){
    Counter c;

    c.inc(1.0);
    c.dec(1.0);
    std::printf("Counter value: %d\n", c.value());

    return 0;
}
