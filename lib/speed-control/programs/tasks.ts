import { Observable, Subscription } from "rxjs";


export class Task {

    tick(incoming_data: any) {
        throw "Not implemented yet";
    }

    private return_promise_resolver: ((value: void | PromiseLike<void>) => void) | null = null;
    private return_promise_rejector: ((reason?: any) => void) | null = null;

    async wait() {
        return this.return_promise;
    }

    async run() {
        this.input_subscription = this.input$.subscribe((data) => this.tick(data));
    }

    return_promise: Promise<void> | null;
    input_subscription: Subscription | null = null;

    input$: Observable<any>;

    constructor(input$: Observable<any>) {
        this.input$ = input$;
        this.return_promise = new Promise<void>((resolve, reject) => {
            this.return_promise_resolver = resolve;
            this.return_promise_rejector = reject;
        }).then((ret: any) => {
            if (this.input_subscription !== null) this.input_subscription.unsubscribe();
            return ret;
        }).catch((err: any) => {
            if (this.input_subscription !== null) this.input_subscription.unsubscribe();
            return err;
        });
    }
}