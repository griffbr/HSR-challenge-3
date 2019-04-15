export declare class Device {
    private _id;
    private _supportedInterfaces;
    id(): string;
    generatedID(): void;
    setID(id: string): void;
    audioPlayerSupported(value?: boolean): boolean;
    displaySupported(value?: boolean): boolean;
    videoAppSupported(value?: boolean): boolean;
    supportedInterfaces(): any;
    private supportedInterface;
}
