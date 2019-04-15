import { SkillContext } from "../core/SkillContext";
export declare class AddressAPI {
    private context;
    constructor(context: SkillContext);
    returnsFullAddress(address: IStreetAddress): void;
    returnsCountryAndPostalCode(address: ICountryAndPostalCode): void;
    insufficientPermissions(): void;
    reset(): void;
    private configure;
}
export interface IAddress {
}
export interface ICountryAndPostalCode extends IAddress {
    countryCode: string;
    postalCode: string;
}
export interface IStreetAddress extends IAddress {
    addressLine1: string;
    addressLine2: string;
    addressLine3: string;
    city: string;
    countryCode: string;
    districtOrCounty: string;
    postalCode: string;
    stateOrRegion: string;
}
