export class CharacteristicMock implements BluetoothRemoteGATTCharacteristic {
    constructor(public readonly service: BluetoothRemoteGATTService, public readonly uuid: string) {}

    properties: BluetoothCharacteristicProperties = {
        // TODO
        broadcast: false,
        read: false,
        writeWithoutResponse: false,
        write: false,
        notify: false,
        indicate: false,
        authenticatedSignedWrites: false,
        reliableWrite: false,
        writableAuxiliaries: false
    }
    value?: DataView<ArrayBufferLike> | undefined;
    getDescriptor(_descriptor: BluetoothDescriptorUUID): Promise<BluetoothRemoteGATTDescriptor> {
        throw new Error("Method not implemented.");
    }
    getDescriptors(_descriptor?: BluetoothDescriptorUUID): Promise<BluetoothRemoteGATTDescriptor[]> {
        throw new Error("Method not implemented.");
    }
    readValue(): Promise<DataView> {
        throw new Error("Method not implemented.");
    }
    async writeValue(_value: BufferSource): Promise<void> {}
    writeValueWithResponse(_value: BufferSource): Promise<void> {
        throw new Error("Method not implemented.");
    }
    writeValueWithoutResponse(_value: BufferSource): Promise<void> {
        throw new Error("Method not implemented.");
    }
    startNotifications(): Promise<BluetoothRemoteGATTCharacteristic> {
        throw new Error("Method not implemented.");
    }
    stopNotifications(): Promise<BluetoothRemoteGATTCharacteristic> {
        throw new Error("Method not implemented.");
    }
    addEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {
        throw new Error("Method not implemented.");
    }
    removeEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {
        throw new Error("Method not implemented.");
    }
    dispatchEvent(_event: unknown): boolean {
        throw new Error("Method not implemented.");
    }
    oncharacteristicvaluechanged: (this: this, ev: Event) => null = () => null

}

export class ServiceMock implements BluetoothRemoteGATTService {
    constructor(public readonly uuid: string, public readonly device: BluetoothDevice, public readonly characteristics?: CharacteristicMock[]) {}
    isPrimary: boolean = true

    getIncludedService(_service: BluetoothServiceUUID): Promise<BluetoothRemoteGATTService> {
        throw new Error("Method not implemented.");
    }
    getIncludedServices(_service?: BluetoothServiceUUID): Promise<BluetoothRemoteGATTService[]> {
        throw new Error("Method not implemented.");
    }
    oncharacteristicvaluechanged: (this: this, ev: Event) => null = () => null;
    onserviceadded: (this: this, ev: Event) => null = () => null;
    onservicechanged: (this: this, ev: Event) => null = () => null;
    onserviceremoved: (this: this, ev: Event) => null = () => null;

    getCharacteristic(characteristic: BluetoothCharacteristicUUID): Promise<BluetoothRemoteGATTCharacteristic> {
        return new Promise((resolve, reject ) => {
            const char = this.characteristics?.find(c => c.uuid === characteristic);
            if (char) {
                resolve(char)
            } else {
                reject()
            }
        });
    }
    getCharacteristics(_characteristic?: BluetoothCharacteristicUUID): Promise<BluetoothRemoteGATTCharacteristic[]> {
        throw new Error("Method not implemented.");
    }
    addEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {
        // throw new Error("Method not implemented.")
    }
    removeEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {
        // throw new Error("Method not implemented.")
    }
    dispatchEvent(_event: unknown): boolean {
        return false;
    }

}

class GattMock implements BluetoothRemoteGATTServer {
    constructor(public readonly device: BluetoothDevice) {}

    services: ServiceMock[] = []
    connected: boolean = true

    async connect(): Promise<BluetoothRemoteGATTServer> {
        return this
    }
    disconnect(): void {
    }
    async getPrimaryService(service: BluetoothServiceUUID): Promise<BluetoothRemoteGATTService> {
        const found = this.services.find(s => s.uuid === service)

        if (found) {
            return found;
        }
        else {
            throw new Error(`Service ${service} not found`);
        }
    }
    getPrimaryServices(_service?: BluetoothServiceUUID): Promise<BluetoothRemoteGATTService[]> {
        throw new Error("Method not implemented.")
    }
}

export class BluetoothDeviceMock implements BluetoothDevice {
    constructor(public readonly id: string, public readonly name?: string) {
        this.gatt = new GattMock(this);
    }

    public addService()

    gatt?: BluetoothRemoteGATTServer | undefined
    watchingAdvertisements: boolean = false

    async forget(): Promise<void> {}
    async watchAdvertisements(_options?: WatchAdvertisementsOptions): Promise<void> {}
    addEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {}
    removeEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {}
    dispatchEvent(_event: unknown): boolean {return false}
    onadvertisementreceived: (this: this, ev: BluetoothAdvertisingEvent) => null = () => null
    ongattserverdisconnected: (this: this, ev: Event) => null = () => null
    oncharacteristicvaluechanged: (this: this, ev: Event) => null = () => null
    onserviceadded: (this: this, ev: Event) => null = () => null
    onservicechanged: (this: this, ev: Event) => null = () => null
    onserviceremoved: (this: this, ev: Event) => null = () => null
}

export class BluetoothMock implements Bluetooth  {
    constructor(private readonly devices: BluetoothDeviceMock[]) {}

    async getDevices(): Promise<BluetoothDevice[]> {
        return this.devices
    }

    async getAvailability(): Promise<boolean> {
        return true
    }
    
    referringDevice?: BluetoothDevice | undefined = undefined

    async requestDevice(_options?: RequestDeviceOptions): Promise<BluetoothDevice> {
        return this.devices[0]
    }
    async requestLEScan(_options?: BluetoothLEScanOptions): Promise<BluetoothLEScan> {
        return {
            active: false,
            stop() {}
        }
    }

    onavailabilitychanged: (this: this, ev: Event) => null = () => null

    addEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {
        
        // throw new Error("Method not implemented.")
    }
    removeEventListener(_type: unknown, _listener: unknown, _options?: unknown): void {
    }
    dispatchEvent(_event: unknown): boolean {
        return false
    }
    onadvertisementreceived: (this: this, ev: BluetoothAdvertisingEvent) => null = () => null
    ongattserverdisconnected: (this: this, ev: Event) => null = () => null
    oncharacteristicvaluechanged: (this: this, ev: Event) => null = () => null
    onserviceadded: (this: this, ev: Event) => null = () => null
    onservicechanged: (this: this, ev: Event) => null = () => null
    onserviceremoved: (this: this, ev: Event) => null = () => null
}